#include "brep.h"
#include "line.h"
#include "mini_test.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "point.h"
#include "polyline.h"
#include "primitives.h"
#include "simple_split.h"
#include "tolerance.h"
#include <cmath>
#include <limits>
#include <stdexcept>
using namespace session_cpp::mini_test;

namespace session_cpp {
MINI_TEST("SimpleSplit", "Split Curve By Curves") {
  auto curve = NurbsCurve::create(false, 1,
                                  {
                                      Point(-2, 0, 0),
                                      Point(2, 0, 0),
                                  });
  auto cutter = NurbsCurve::create(false, 1,
                                   {
                                       Point(0, -2, 0),
                                       Point(0, 2, 0),
                                   });
  auto pieces = simple_split::split_curve_by_curves(curve, {cutter}, 1e-6);
  MINI_CHECK(pieces.size() == 2);
  MINI_CHECK(pieces[0].point_at_start().distance(Point(-2, 0, 0)) < 1e-6);
  MINI_CHECK(pieces[0].point_at_end().distance(Point(0, 0, 0)) < 1e-6);
  MINI_CHECK(pieces[1].point_at_end().distance(Point(2, 0, 0)) < 1e-6);
  MINI_CHECK(curve.point_at_end().distance(Point(2, 0, 0)) < 1e-6);
  auto skew = NurbsCurve::create(false, 1,
                                 {
                                     Point(0, -2, 1),
                                     Point(0, 2, 1),
                                 });
  MINI_CHECK(simple_split::split_curve_by_curves(curve, {skew}, 1e-6).size() ==
             1);
  auto crossing = NurbsCurve::create(
      false, 1,
      {Point(-2, -2, 0), Point(2, 2, 0), Point(-2, 2, 0), Point(2, -2, 0)});
  auto short_cut =
      NurbsCurve::create(false, 1, {Point(0, -.2, 0), Point(0, .2, 0)});
  MINI_CHECK(
      simple_split::split_curve_by_curves(crossing, {short_cut}, 1e-6).size() ==
      3);
  auto circle = Primitives::circle(0, 0, 0, 1);
  auto chord = NurbsCurve::create(false, 1,
                                  {
                                      Point(-2, 0.5, 0),
                                      Point(2, 0.5, 0),
                                  });
  auto arcs = simple_split::split_curve_by_curves(circle, {chord}, 1e-6);
  MINI_CHECK(arcs.size() == 2);
  MINI_CHECK(arcs[0].is_rational() && arcs[1].is_rational());
  auto tangent = NurbsCurve::create(false, 1,
                                    {
                                        Point(-2, 1, 0),
                                        Point(2, 1, 0),
                                    });
  MINI_CHECK(
      simple_split::split_curve_by_curves(circle, {tangent}, 1e-6).size() == 1);
  bool rejected = false;
  try {
    simple_split::split_curve_by_curves(curve, {curve}, 1e-6);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  MINI_CHECK(rejected);
  rejected = false;
  try {
    simple_split::split_curve_by_curves(
        curve, {cutter}, std::numeric_limits<double>::quiet_NaN());
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  MINI_CHECK(rejected);
}

MINI_TEST("SimpleSplit", "Split BRep Face By Curves") {
  auto box = BRep::create_box(10, 10, 10);
  auto surface = box.m_surfaces[0];
  auto a = surface.get_cv(0, 0);
  auto u = surface.get_cv(1, 0);
  auto v = surface.get_cv(0, 1);
  auto mapped = [&](double x, double y) {
    return Point(a[0] + x * (u[0] - a[0]) + y * (v[0] - a[0]),
                 a[1] + x * (u[1] - a[1]) + y * (v[1] - a[1]),
                 a[2] + x * (u[2] - a[2]) + y * (v[2] - a[2]));
  };
  auto cutter = NurbsCurve::create(false, 1,
                                   {
                                       mapped(0.5, -1),
                                       mapped(0.5, 2),
                                   });
  auto split = simple_split::split_brep_face_by_curves(box, 0, {cutter}, 1e-6);
  MINI_CHECK(split.face_count() == 7);
  MINI_CHECK(split.is_valid() && split.is_solid());
  MINI_CHECK(box.face_count() == 6);
  auto meshes = split.face_meshes_q(true, 20., 0.005);
  MINI_CHECK(std::abs(meshes[0].area() - 50.) < 1e-6);
  MINI_CHECK(std::abs(meshes[6].area() - 50.) < 1e-6);
  double neighbor_area = 0.;
  for (int i = 1; i < 6; ++i)
    neighbor_area += meshes[i].area();
  MINI_CHECK(std::abs(neighbor_area - 500.) < 1e-6);
  auto closed = NurbsCurve::create(
      false, 3,
      {mapped(.2, .3), mapped(.8, .3), mapped(.5, .9), mapped(.2, .3)});
  auto island = simple_split::split_brep_face_by_curves(box, 0, {closed}, 1e-6);
  MINI_CHECK(island.face_count() == 7 && island.is_solid());

  auto crossing = NurbsCurve::create(false, 1,
                                     {
                                         mapped(-1, 0.5),
                                         mapped(2, 0.5),
                                     });
  auto quarters =
      simple_split::split_brep_face_by_curves(box, 0, {cutter, crossing}, 1e-6);
  MINI_CHECK(quarters.face_count() == 9 && quarters.is_solid());
  auto repeated =
      simple_split::split_brep_face_by_curves(split, 0, {crossing}, 1e-6);
  MINI_CHECK(repeated.face_count() == 8 && repeated.is_solid());
  auto loop = NurbsCurve::create(false, 1,
                                 {
                                     mapped(0.2, 0.2),
                                     mapped(0.8, 0.2),
                                     mapped(0.8, 0.8),
                                     mapped(0.2, 0.8),
                                     mapped(0.2, 0.2),
                                 });
  auto regions = simple_split::split_brep_face_by_curves(box, 0, {loop}, 1e-6);
  MINI_CHECK(regions.face_count() == 7 && regions.is_solid());
  auto restored = BRep::file_json_loads(quarters.file_json_dumps());
  MINI_CHECK(restored.face_count() == 9 && restored.is_solid());
  auto protobuf = BRep::pb_loads(quarters.pb_dumps());
  MINI_CHECK(protobuf.face_count() == 9 && protobuf.is_solid());
  auto outer = NurbsCurve::create(false, 1,
                                  {
                                      Point(0, 0, 0),
                                      Point(10, 0, 0),
                                      Point(10, 10, 0),
                                      Point(0, 10, 0),
                                      Point(0, 0, 0),
                                  });
  auto inner = NurbsCurve::create(false, 1,
                                  {
                                      Point(3, 3, 0),
                                      Point(7, 3, 0),
                                      Point(7, 7, 0),
                                      Point(3, 7, 0),
                                      Point(3, 3, 0),
                                  });
  auto ring = BRep::from_nurbscurves({outer}, {{inner}});
  auto through = NurbsCurve::create(false, 1,
                                    {
                                        Point(5, -1, 0),
                                        Point(5, 11, 0),
                                    });
  auto divided =
      simple_split::split_brep_face_by_curves(ring, 0, {through}, 1e-6);
  MINI_CHECK(divided.face_count() == 2);
  auto outside = NurbsCurve::create(false, 1,
                                    {
                                        Point(1, -1, 0),
                                        Point(1, 11, 0),
                                    });
  auto preserved =
      simple_split::split_brep_face_by_curves(ring, 0, {outside}, 1e-6);
  int holes = 0;
  for (const auto &face : preserved.m_faces)
    holes += static_cast<int>(face.wires.size()) - 1;
  MINI_CHECK(preserved.face_count() == 2 && holes == 1);
  auto disk = BRep::from_nurbscurves({Primitives::circle(0, 0, 0, 5)});
  auto chord = NurbsCurve::create(false, 1,
                                  {
                                      Point(-6, 1.2, 0),
                                      Point(6, 1.2, 0),
                                  });
  auto halves = simple_split::split_brep_face_by_curves(disk, 0, {chord}, 1e-6);
  MINI_CHECK(halves.face_count() == 2);
  MINI_CHECK(disk.face_count() == 1);
  auto cylinder = BRep::create_cylinder(5, 10);
  auto body = cylinder.m_surfaces[cylinder.m_faces[0].surface_index];
  auto domain = body.domain(0);
  auto generator = body.iso_curve(1, (domain.first + domain.second) * 0.5);
  auto seamed =
      simple_split::split_brep_face_by_curves(cylinder, 0, {generator}, 1e-6);
  MINI_CHECK(seamed.face_count() == 4 && seamed.is_solid());
  MINI_CHECK(cylinder.face_count() == 3);
}

MINI_TEST("SimpleSplit", "Split Surface By Curves") {
  auto surface = BRep::create_box(10, 10, 10).m_surfaces[0];
  auto a = surface.get_cv(0, 0);
  auto u = surface.get_cv(1, 0);
  auto v = surface.get_cv(0, 1);
  auto mapped = [&](double x, double y) {
    return Point(a[0] + x * (u[0] - a[0]) + y * (v[0] - a[0]),
                 a[1] + x * (u[1] - a[1]) + y * (v[1] - a[1]),
                 a[2] + x * (u[2] - a[2]) + y * (v[2] - a[2]));
  };
  auto cutter = NurbsCurve::create(false, 1,
                                   {
                                       mapped(0.5, -1),
                                       mapped(0.5, 2),
                                   });
  auto split = simple_split::split_surface_by_curves(surface, {cutter}, 1e-6);
  MINI_CHECK(split.face_count() == 2);
  MINI_CHECK(split.is_valid() && !split.is_solid());
  auto outside = NurbsCurve::create(false, 1,
                                    {
                                        mapped(2, -1),
                                        mapped(2, 2),
                                    });
  auto untouched =
      simple_split::split_surface_by_curves(surface, {outside}, 1e-6);
  MINI_CHECK(untouched.face_count() == 1);
  MINI_CHECK(surface.is_valid());
  auto invalid = surface;
  invalid.set_cv(0, 0, Point(std::numeric_limits<double>::quiet_NaN(), 0, 0));
  bool rejected = false;
  try {
    simple_split::split_surface_by_curves(invalid, {outside}, 1e-6);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  MINI_CHECK(rejected);
}

MINI_TEST("SimpleSplit", "Split Line By Curves") {
  auto line = Line::from_points(Point(-2, 0, 0), Point(2, 0, 0));
  line.name = "retained";
  line.width = 3.;
  line.dash = {1., 2.};
  auto cutter = NurbsCurve::create(false, 1, {Point(0, -2, 0), Point(0, 2, 0)});
  auto pieces = simple_split::split_line_by_curves(line, {cutter}, 1e-6);
  MINI_CHECK(pieces.size() == 2);
  MINI_CHECK(pieces[0].point_at(1).distance(Point(0, 0, 0)) < 1e-6);
  MINI_CHECK(pieces[1].point_at(0).distance(Point(0, 0, 0)) < 1e-6);
  MINI_CHECK(pieces[0].name == line.name && pieces[0].width == line.width &&
             pieces[0].dash == line.dash);
  MINI_CHECK(line.length() == 4.);
}
MINI_TEST("SimpleSplit", "Split Polyline By Curves") {
  Polyline polyline({Point(-2, 0, 0), Point(2, 0, 0), Point(2, 3, 0)});
  polyline.name = "retained";
  polyline.width = 3.;
  polyline.dash = {1., 2.};
  auto cutter = NurbsCurve::create(false, 1, {Point(0, -2, 0), Point(0, 2, 0)});
  auto pieces =
      simple_split::split_polyline_by_curves(polyline, {cutter}, 1e-6);
  MINI_CHECK(pieces.size() == 2);
  MINI_CHECK(pieces[0].point_count() == 2 && pieces[1].point_count() == 3);
  MINI_CHECK(pieces[1].get_point(1).distance(Point(2, 0, 0)) < 1e-6);
  MINI_CHECK(pieces[1].get_point(2).distance(Point(2, 3, 0)) < 1e-6);
  MINI_CHECK(pieces[0].name == polyline.name &&
             pieces[0].width == polyline.width &&
             pieces[0].dash == polyline.dash);
  MINI_CHECK(polyline.point_count() == 3);
}

} // namespace session_cpp
