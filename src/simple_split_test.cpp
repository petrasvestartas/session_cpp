#include "mini_test.h"
#include "simple_split.h"
#include "brep.h"
#include "line.h"
#include "mesh.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "point.h"
#include "polyline.h"
#include "primitives.h"
#include "tolerance.h"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

/// Point at fractions x and y along the first two control edges of a surface.
static Point mapped(const NurbsSurface& surface, double x, double y) {

    const Point a = surface.get_cv(0, 0);
    return a + (surface.get_cv(1, 0) - a) * x + (surface.get_cv(0, 1) - a) * y;
}

MINI_TEST("SimpleSplit", "Split Curve By Curves") {

    const NurbsCurve curve = NurbsCurve::create(false, 1, {Point(-2.0, 0.0, 0.0), Point(2.0, 0.0, 0.0)});
    const NurbsCurve cutter = NurbsCurve::create(false, 1, {Point(0.0, -2.0, 0.0), Point(0.0, 2.0, 0.0)});
    const std::vector<NurbsCurve> pieces = simple_split::split_curve_by_curves(curve, {cutter}, 1e-6);

    MINI_CHECK(pieces.size() == 2);
    MINI_CHECK(pieces[0].point_at_start().distance(Point(-2.0, 0.0, 0.0)) < 1e-6);
    MINI_CHECK(pieces[0].point_at_end().distance(Point(0.0, 0.0, 0.0)) < 1e-6);
    MINI_CHECK(pieces[1].point_at_end().distance(Point(2.0, 0.0, 0.0)) < 1e-6);
    MINI_CHECK(curve.point_at_end().distance(Point(2.0, 0.0, 0.0)) < 1e-6);

    const NurbsCurve skew = NurbsCurve::create(false, 1, {Point(0.0, -2.0, 1.0), Point(0.0, 2.0, 1.0)});

    MINI_CHECK(simple_split::split_curve_by_curves(curve, {skew}, 1e-6).size() == 1);

    const NurbsCurve crossing = NurbsCurve::create(
        false,
        1,
        {
            Point(-2.0, -2.0, 0.0),
            Point(2.0, 2.0, 0.0),
            Point(-2.0, 2.0, 0.0),
            Point(2.0, -2.0, 0.0),
        }
    );
    const NurbsCurve short_cut = NurbsCurve::create(false, 1, {Point(0.0, -0.2, 0.0), Point(0.0, 0.2, 0.0)});

    MINI_CHECK(simple_split::split_curve_by_curves(crossing, {short_cut}, 1e-6).size() == 3);

    const NurbsCurve circle = Primitives::circle(0.0, 0.0, 0.0, 1.0);
    const NurbsCurve chord = NurbsCurve::create(false, 1, {Point(-2.0, 0.5, 0.0), Point(2.0, 0.5, 0.0)});
    const std::vector<NurbsCurve> arcs = simple_split::split_curve_by_curves(circle, {chord}, 1e-6);

    MINI_CHECK(arcs.size() == 2);
    MINI_CHECK(arcs[0].is_rational() && arcs[1].is_rational());

    const NurbsCurve tangent = NurbsCurve::create(false, 1, {Point(-2.0, 1.0, 0.0), Point(2.0, 1.0, 0.0)});

    MINI_CHECK(simple_split::split_curve_by_curves(circle, {tangent}, 1e-6).size() == 1);

    bool rejected = false;

    try {
        simple_split::split_curve_by_curves(curve, {curve}, 1e-6);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    MINI_CHECK(rejected);

    rejected = false;

    try {
        simple_split::split_curve_by_curves(curve, {cutter}, std::numeric_limits<double>::quiet_NaN());
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    MINI_CHECK(rejected);
}

MINI_TEST("SimpleSplit", "Split BRep Face By Curves") {

    const BRep box = BRep::create_box(10.0, 10.0, 10.0);
    const NurbsSurface surface = box.m_surfaces[0];
    const NurbsCurve cutter = NurbsCurve::create(false, 1, {mapped(surface, 0.5, -1.0), mapped(surface, 0.5, 2.0)});
    const BRep split = simple_split::split_brep_face_by_curves(box, 0, {cutter}, 1e-6);

    MINI_CHECK(split.face_count() == 7);
    MINI_CHECK(split.is_valid() && split.is_solid());
    MINI_CHECK(box.face_count() == 6);

    const std::vector<Mesh> meshes = split.face_meshes_q(true, 20.0, 0.005);

    MINI_CHECK(std::abs(meshes[0].area() - 50.0) < 1e-6);
    MINI_CHECK(std::abs(meshes[6].area() - 50.0) < 1e-6);

    double neighbor_area = 0.0;

    for (int i = 1; i < 6; ++i)
        neighbor_area += meshes[i].area();

    MINI_CHECK(std::abs(neighbor_area - 500.0) < 1e-6);

    const NurbsCurve closed = NurbsCurve::create(
        false,
        3,
        {
            mapped(surface, 0.2, 0.3),
            mapped(surface, 0.8, 0.3),
            mapped(surface, 0.5, 0.9),
            mapped(surface, 0.2, 0.3),
        }
    );
    const BRep island = simple_split::split_brep_face_by_curves(box, 0, {closed}, 1e-6);

    MINI_CHECK(island.face_count() == 7 && island.is_solid());

    const NurbsCurve crossing = NurbsCurve::create(false, 1, {mapped(surface, -1.0, 0.5), mapped(surface, 2.0, 0.5)});
    const BRep quarters = simple_split::split_brep_face_by_curves(box, 0, {cutter, crossing}, 1e-6);

    MINI_CHECK(quarters.face_count() == 9 && quarters.is_solid());

    const BRep repeated = simple_split::split_brep_face_by_curves(split, 0, {crossing}, 1e-6);

    MINI_CHECK(repeated.face_count() == 8 && repeated.is_solid());

    const NurbsCurve loop = NurbsCurve::create(
        false,
        1,
        {
            mapped(surface, 0.2, 0.2),
            mapped(surface, 0.8, 0.2),
            mapped(surface, 0.8, 0.8),
            mapped(surface, 0.2, 0.8),
            mapped(surface, 0.2, 0.2),
        }
    );
    const BRep regions = simple_split::split_brep_face_by_curves(box, 0, {loop}, 1e-6);

    MINI_CHECK(regions.face_count() == 7 && regions.is_solid());

    const BRep restored = BRep::file_json_loads(quarters.file_json_dumps());

    MINI_CHECK(restored.face_count() == 9 && restored.is_solid());

    const BRep protobuf = BRep::pb_loads(quarters.pb_dumps());

    MINI_CHECK(protobuf.face_count() == 9 && protobuf.is_solid());

    const NurbsCurve outer = NurbsCurve::create(
        false,
        1,
        {
            Point(0.0, 0.0, 0.0),
            Point(10.0, 0.0, 0.0),
            Point(10.0, 10.0, 0.0),
            Point(0.0, 10.0, 0.0),
            Point(0.0, 0.0, 0.0),
        }
    );
    const NurbsCurve inner = NurbsCurve::create(
        false,
        1,
        {
            Point(3.0, 3.0, 0.0),
            Point(7.0, 3.0, 0.0),
            Point(7.0, 7.0, 0.0),
            Point(3.0, 7.0, 0.0),
            Point(3.0, 3.0, 0.0),
        }
    );
    const BRep ring = BRep::from_nurbscurves({outer}, {{inner}});
    const NurbsCurve through = NurbsCurve::create(false, 1, {Point(5.0, -1.0, 0.0), Point(5.0, 11.0, 0.0)});
    const BRep divided = simple_split::split_brep_face_by_curves(ring, 0, {through}, 1e-6);

    MINI_CHECK(divided.face_count() == 2);

    const NurbsCurve outside = NurbsCurve::create(false, 1, {Point(1.0, -1.0, 0.0), Point(1.0, 11.0, 0.0)});
    const BRep preserved = simple_split::split_brep_face_by_curves(ring, 0, {outside}, 1e-6);
    int holes = 0;

    for (const BRepFace& face : preserved.m_faces)
        holes += static_cast<int>(face.wires.size()) - 1;

    MINI_CHECK(preserved.face_count() == 2 && holes == 1);

    const BRep disk = BRep::from_nurbscurves({Primitives::circle(0.0, 0.0, 0.0, 5.0)});
    const NurbsCurve chord = NurbsCurve::create(false, 1, {Point(-6.0, 1.2, 0.0), Point(6.0, 1.2, 0.0)});
    const BRep halves = simple_split::split_brep_face_by_curves(disk, 0, {chord}, 1e-6);

    MINI_CHECK(halves.face_count() == 2);
    MINI_CHECK(disk.face_count() == 1);

    const BRep cylinder = BRep::create_cylinder(5.0, 10.0);
    const NurbsSurface body = cylinder.m_surfaces[cylinder.m_faces[0].surface_index];
    const std::pair<double, double> domain = body.domain(0);
    const NurbsCurve generator = body.iso_curve(1, (domain.first + domain.second) * 0.5);
    const BRep seamed = simple_split::split_brep_face_by_curves(cylinder, 0, {generator}, 1e-6);

    MINI_CHECK(seamed.face_count() == 4 && seamed.is_solid());
    MINI_CHECK(cylinder.face_count() == 3);
}

MINI_TEST("SimpleSplit", "Split Surface By Curves") {

    const NurbsSurface surface = BRep::create_box(10.0, 10.0, 10.0).m_surfaces[0];
    const NurbsCurve cutter = NurbsCurve::create(false, 1, {mapped(surface, 0.5, -1.0), mapped(surface, 0.5, 2.0)});
    const BRep split = simple_split::split_surface_by_curves(surface, {cutter}, 1e-6);

    MINI_CHECK(split.face_count() == 2);
    MINI_CHECK(split.is_valid() && !split.is_solid());

    const NurbsCurve outside = NurbsCurve::create(false, 1, {mapped(surface, 2.0, -1.0), mapped(surface, 2.0, 2.0)});
    const BRep untouched = simple_split::split_surface_by_curves(surface, {outside}, 1e-6);

    MINI_CHECK(untouched.face_count() == 1);
    MINI_CHECK(surface.is_valid());

    NurbsSurface invalid = surface;
    invalid.set_cv(0, 0, Point(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
    bool rejected = false;

    try {
        simple_split::split_surface_by_curves(invalid, {outside}, 1e-6);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    MINI_CHECK(rejected);
}

MINI_TEST("SimpleSplit", "Split Line By Curves") {

    Line line = Line::from_points(Point(-2.0, 0.0, 0.0), Point(2.0, 0.0, 0.0));
    line.name = "retained";
    line.width = 3.0;
    line.dash = {1.0, 2.0};
    const NurbsCurve cutter = NurbsCurve::create(false, 1, {Point(0.0, -2.0, 0.0), Point(0.0, 2.0, 0.0)});
    const std::vector<Line> pieces = simple_split::split_line_by_curves(line, {cutter}, 1e-6);

    MINI_CHECK(pieces.size() == 2);
    MINI_CHECK(pieces[0].point_at(1.0).distance(Point(0.0, 0.0, 0.0)) < 1e-6);
    MINI_CHECK(pieces[1].point_at(0.0).distance(Point(0.0, 0.0, 0.0)) < 1e-6);
    MINI_CHECK(pieces[0].name == line.name && pieces[0].width == line.width && pieces[0].dash == line.dash);
    MINI_CHECK(line.length() == 4.0);
}

MINI_TEST("SimpleSplit", "Split Polyline By Curves") {

    Polyline polyline({Point(-2.0, 0.0, 0.0), Point(2.0, 0.0, 0.0), Point(2.0, 3.0, 0.0)});
    polyline.name = "retained";
    polyline.width = 3.0;
    polyline.dash = {1.0, 2.0};
    const NurbsCurve cutter = NurbsCurve::create(false, 1, {Point(0.0, -2.0, 0.0), Point(0.0, 2.0, 0.0)});
    const std::vector<Polyline> pieces = simple_split::split_polyline_by_curves(polyline, {cutter}, 1e-6);

    MINI_CHECK(pieces.size() == 2);
    MINI_CHECK(pieces[0].point_count() == 2 && pieces[1].point_count() == 3);
    MINI_CHECK(pieces[1].get_point(1).distance(Point(2.0, 0.0, 0.0)) < 1e-6);
    MINI_CHECK(pieces[1].get_point(2).distance(Point(2.0, 3.0, 0.0)) < 1e-6);
    MINI_CHECK(pieces[0].name == polyline.name && pieces[0].width == polyline.width && pieces[0].dash == polyline.dash);
    MINI_CHECK(polyline.point_count() == 3);
}

} // namespace session_cpp
