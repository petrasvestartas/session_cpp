#include "brep_classify.h"
#include "brep_face_boundary.h"
#include "nurbs_memo.h"
#include "primitives.h"
#include "file_step.h"
#include "mini_test.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>

using namespace session_cpp;
using namespace session_cpp::mini_test;

namespace {

void save_failure(const BRep& shape, const Point& query, double tolerance, brep_classify::State state) {
    const std::array<Vector, 7> directions = {
        Vector(1.0, 0.371, 0.529), Vector(-0.217, 1.0, 0.413), Vector(0.319, -0.271, 1.0),
        Vector(-1.0, 0.723, -0.219), Vector(0.613, -1.0, -0.417), Vector(-0.531, -0.337, -1.0), Vector(1.0, 1.0, 1.0)
    };
    nlohmann::ordered_json rays = nlohmann::ordered_json::array();
    for (const Vector& direction : directions) {
        const auto result = brep_classify::ray(shape, query, direction, tolerance);
        nlohmann::ordered_json hits = nlohmann::ordered_json::array();
        for (const auto& hit : result.hits)
            hits.push_back({{"face", hit.face}, {"u", hit.u}, {"v", hit.v}, {"distance", hit.distance},
                {"distance_enclosure", hit.distance_enclosure}, {"crossing", hit.crossing}, {"trim", static_cast<int>(hit.trim)}});
        rays.push_back({{"direction", {direction[0], direction[1], direction[2]}}, {"complete", result.complete},
            {"failure", result.failure}, {"unresolved_face", result.unresolved_face},
            {"unresolved_domain", result.unresolved_domain}, {"hits", hits}});
    }
    const nlohmann::ordered_json report = {{"shape", shape.jsondump()}, {"query", {query[0], query[1], query[2]}},
        {"tolerance", tolerance}, {"state", static_cast<int>(state)}, {"rays", rays}};
    const auto folder = std::filesystem::path(__FILE__).parent_path().parent_path() / "build/native-geometry-tests";
    std::filesystem::create_directories(folder);
    std::ofstream(folder / "classification_bore_origin_failure.json") << report.dump(2);
}

}

MINI_TEST("BRep Native Classification", "Narrow Parameter Subdivision") {
    const NurbsCurve source = NurbsCurve::create(false, 2, {{0, 0, 0}, {1, 2, 0}, {2, 0, 0}});
    for (double width : {1e-6, 1e-9, 1e-12}) {
        NurbsCurve curve = source;
        MINI_CHECK(curve.trim(0.5, 0.5 + width));
        MINI_CHECK(curve.domain_start() == 0.5 && curve.domain_end() == 0.5 + width);
        const std::pair<NurbsCurve, NurbsCurve> pieces = curve.split(0.5 + width * 0.5);
        MINI_CHECK(pieces.first.is_valid() && pieces.second.is_valid());
        MINI_CHECK(pieces.first.point_at_end().distance(source.point_at(0.5 + width * 0.5)) < 1e-12);
        MINI_CHECK(pieces.second.point_at_start().distance(pieces.first.point_at_end()) < 1e-12);
    }
    NurbsCurve tiny = source;
    MINI_CHECK(tiny.set_domain(0.0, 1e-12));
    MINI_CHECK(tiny.trim(2e-13, 7e-13));
    MINI_CHECK(tiny.point_at_start().distance(source.point_at(source.domain_end() * 0.2)) < 1e-12);
    MINI_CHECK(tiny.point_at_end().distance(source.point_at(source.domain_end() * 0.7)) < 1e-12);
}

MINI_TEST("BRep Native Classification", "Box And Boundary") {
    const BRep box = BRep::create_box(2, 2, 2);
    MINI_CHECK(brep_classify::point(box, {0, 0, 0}, 1e-7) == brep_classify::State::Inside);
    MINI_CHECK(brep_classify::point(box, {2, 0, 0}, 1e-7) == brep_classify::State::Outside);
    MINI_CHECK(brep_classify::point(box, {1, 0, 0}, 1e-7) == brep_classify::State::Boundary);
    MINI_CHECK(brep_classify::point(box, {1, 1, 0}, 1e-7) == brep_classify::State::Boundary);
    MINI_CHECK(brep_classify::point(box, {1, 1, 1}, 1e-7) == brep_classify::State::Boundary);
    MINI_CHECK(brep_classify::point(BRep(), {0, 0, 0}, 1e-7) == brep_classify::State::Outside);
}

MINI_TEST("BRep Native Classification", "Grazing Ray Has One Root Per Face") {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_rotated_cube_classification_failure.json");
    MINI_CHECK(input.good());
    const auto fixture = nlohmann::json::parse(input);
    const BRep source = BRep::jsonload(fixture["shape"]);
    const Point original(fixture["query"][0], fixture["query"][1], fixture["query"][2]);
    const double reference = fixture["unique_grazing_root_distance"];
    for (double scale : {0.001, 1.0, 1000.0}) {
        const Xform transform = Xform::translation(31 * scale, -17 * scale, 13 * scale)
            * Xform::scale_xyz(scale, scale, scale);
        const BRep shape = source.transformed(transform);
        const Point query = original.transformed(transform);
        const double tolerance = scale * 1e-7;
        const auto ray = brep_classify::ray(shape, query, {-0.217, 1, 0.413}, tolerance);
        MINI_CHECK(ray.complete && ray.hits.size() == 2);
        MINI_CHECK(ray.hits[0].face == 4 && ray.hits[0].crossing == -1 && ray.hits[1].crossing == 1);
        MINI_CHECK(std::abs(ray.hits[0].distance - scale * reference) <= tolerance);
        MINI_CHECK(brep_classify::point(shape, query, tolerance) == brep_classify::State::Outside);
    }
}

MINI_TEST("BRep Native Classification", "Boundary Tolerance Band") {
    const BRep box = BRep::create_box(2, 2, 2);
    for (int axis = 0; axis < 3; ++axis)
        for (double sign : {-1.0, 1.0})
            for (double offset : {-0.5e-6, 0.5e-6}) {
                Point query;
                query[axis] = sign * (1.0 + offset);
                MINI_CHECK(brep_classify::point(box, query, 1e-6) == brep_classify::State::Boundary);
                query[axis] = sign * (1.0 + offset * 4.0);
                const brep_classify::State expected = offset < 0.0 ? brep_classify::State::Inside : brep_classify::State::Outside;
                MINI_CHECK(brep_classify::point(box, query, 1e-6) == expected);
            }
    const BRep sphere = BRep::create_sphere(1.0);
    MINI_CHECK(brep_classify::point(sphere, {0, 0, 1.0 + 0.5e-6}, 1e-6) == brep_classify::State::Boundary);
    MINI_CHECK(brep_classify::point(sphere, {0, 0, 1.0 + 2e-6}, 1e-6) == brep_classify::State::Outside);
}

MINI_TEST("BRep Native Classification", "Rational Sphere Rays") {
    const BRep sphere = BRep::create_sphere(1.0);
    const brep_classify::RayResult ray = brep_classify::ray(sphere, {-2, 0.13, 0.21}, {3, 0, 0}, 1e-7);
    if (!ray.complete)
        throw std::runtime_error(ray.failure);
    MINI_CHECK(ray.complete);
    MINI_CHECK(ray.hits.size() == 2);
    const double x = std::sqrt(1.0 - 0.13 * 0.13 - 0.21 * 0.21);
    MINI_CHECK(std::abs(ray.hits[0].distance - (2.0 - x)) < 1e-7);
    MINI_CHECK(std::abs(ray.hits[1].distance - (2.0 + x)) < 1e-7);
    MINI_CHECK(ray.hits[0].crossing == -1 && ray.hits[1].crossing == 1);
    MINI_CHECK(brep_classify::point(sphere, {0, 0, 0}, 1e-7) == brep_classify::State::Inside);
    MINI_CHECK(brep_classify::point(sphere, {0, 0, 1.1}, 1e-7) == brep_classify::State::Outside);
    MINI_CHECK(brep_classify::point(sphere, {0, 0, 1}, 1e-7) == brep_classify::State::Boundary);
    MINI_CHECK(brep_classify::point(sphere, {1, 0, 0}, 1e-7) == brep_classify::State::Boundary);
}

MINI_TEST("BRep Native Classification", "Bore Trims") {
    const BRep solid = BRep::create_block_with_hole(4, 4, 2, 1);
    const brep_classify::RayResult bore = brep_classify::ray(solid, {0, 0, -3}, {0, 0, 1}, 1e-7);
    if (!bore.complete)
        throw std::runtime_error(bore.failure);
    MINI_CHECK(bore.complete && bore.hits.empty());
    const brep_classify::RayResult wall = brep_classify::ray(solid, {-3, 0.2, 0}, {1, 0, 0}, 1e-7);
    MINI_CHECK(wall.complete && wall.hits.size() == 4);
    MINI_CHECK(brep_classify::point(solid, {0, 0, 0}, 1e-7) == brep_classify::State::Outside);
    MINI_CHECK(brep_classify::point(solid, {1.5, 0, 0}, 1e-7) == brep_classify::State::Inside);
    MINI_CHECK(brep_classify::point(solid, {1, 0, 0}, 1e-7) == brep_classify::State::Boundary);
    const auto state = brep_classify::point(solid, {0, 0, 1}, 1e-7);
    if (state != brep_classify::State::Outside)
        save_failure(solid, {0, 0, 1}, 1e-7, state);
    MINI_CHECK(state == brep_classify::State::Outside);
    MINI_CHECK(brep_classify::point(solid, {1.5, 0, 1}, 1e-7) == brep_classify::State::Boundary);
}

MINI_TEST("BRep Native Classification", "Thin Torus And Cone") {
    const BRep torus = BRep::create_torus(2.0, 0.01);
    MINI_CHECK(brep_classify::point(torus, {0, 0, 0}, 1e-7) == brep_classify::State::Outside);
    MINI_CHECK(brep_classify::point(torus, {2, 0, 0}, 1e-7) == brep_classify::State::Inside);
    MINI_CHECK(brep_classify::point(torus, {2, 0, 0.02}, 1e-7) == brep_classify::State::Outside);
    MINI_CHECK(brep_classify::point(torus, {2.01, 0, 0}, 1e-7) == brep_classify::State::Boundary);
    const BRep cone = BRep::create_cone(1.0, 2.0);
    MINI_CHECK(brep_classify::point(cone, {0, 0, 1}, 1e-7) == brep_classify::State::Inside);
    MINI_CHECK(brep_classify::point(cone, {0, 0, 2}, 1e-7) == brep_classify::State::Boundary);
    MINI_CHECK(brep_classify::point(cone, {0.75, 0, 1}, 1e-7) == brep_classify::State::Outside);
}

MINI_TEST("BRep Native Classification", "Recognised Forms Kept On The Solid") {
    const BRep torus = BRep::create_torus(0.8, 0.25);
    const brep_classify::Solid solid(torus);
    MINI_CHECK(solid.forms.size() == 1 && !solid.forms[0]);
    MINI_CHECK(brep_classify::point(solid, {0.8, 0.1, 0.05}, 1e-7) == brep_classify::State::Inside);
    MINI_CHECK(solid.forms[0] && solid.forms[0]->implicit.kind == 5);
    const surface_ray::Form* kept = &*solid.forms[0];
    MINI_CHECK(brep_classify::point(solid, {0.1, 0.1, 0}, 1e-7) == brep_classify::State::Outside);
    MINI_CHECK(brep_classify::point(solid, {0.3, -0.7, 0.2}, 1e-7) == brep_classify::State::Inside);
    MINI_CHECK(brep_classify::point(solid, {0.3, -0.7, 0.3}, 1e-7) == brep_classify::State::Outside);
    MINI_CHECK(&*solid.forms[0] == kept);
}

MINI_TEST("BRep Native Classification", "Recognised Forms Laid Out Once In A Scope") {
    const BRep sphere = BRep::create_sphere(0.8), copy = sphere;
    const nurbs_memo::Scope scope;
    const brep_classify::Solid first(sphere), second(copy);
    MINI_CHECK(brep_classify::point(first, {0.1, 0.2, 0.3}, 1e-7) == brep_classify::State::Inside);
    MINI_CHECK(brep_classify::point(second, {0.9, 0.2, 0.3}, 1e-7) == brep_classify::State::Outside);
    // One surface of the same bytes, one form for both models, none kept on either.
    MINI_CHECK(scope.forms.size() == 1 && scope.forms.begin()->second->form && scope.forms.begin()->second->form->implicit.kind == 2);
    MINI_CHECK(!first.forms[0] && !second.forms[0]);
}

MINI_TEST("BRep Native Classification", "Scale And Translation") {
    for (double scale : {0.001, 1.0, 1000.0}) {
        const BRep sphere = BRep::create_sphere(scale);
        MINI_CHECK(brep_classify::point(sphere, {0.25 * scale, 0, 0}, scale * 1e-7) == brep_classify::State::Inside);
        MINI_CHECK(brep_classify::point(sphere, {1.01 * scale, 0, 0}, scale * 1e-7) == brep_classify::State::Outside);
    }
    const BRep shifted = BRep::create_sphere(1.0).transformed(Xform::translation(1e6, -1e6, 1e6));
    MINI_CHECK(brep_classify::point(shifted, {1e6, -1e6, 1e6}, 1e-6) == brep_classify::State::Inside);
    MINI_CHECK(brep_classify::point(shifted, {1e6 + 1.01, -1e6, 1e6}, 1e-6) == brep_classify::State::Outside);
    MINI_CHECK(brep_classify::point(shifted, {1e6, -1e6, 1e6}, 1e-12) == brep_classify::State::Unknown);
}

MINI_TEST("BRep Native Classification", "Invalid Inputs") {
    const BRep box = BRep::create_box(2, 2, 2);
    for (double tolerance : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected = false;
        try {
            static_cast<void>(brep_classify::point(box, {0, 0, 0}, tolerance));
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        MINI_CHECK(rejected);
    }
    BRep open = box;
    open.m_shells[0].faces.pop_back();
    bool rejected = false;
    try {
        static_cast<void>(brep_classify::point(open, {0, 0, 0}, 1e-7));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    MINI_CHECK(rejected);
}

MINI_TEST("BRep Native Classification", "Cavity And Unused Faces") {
    std::vector<Polyline> faces = BRep::create_box(4, 4, 4).face_polylines();
    const std::vector<Polyline> inner = BRep::create_box(2, 2, 2).face_polylines();
    faces.insert(faces.end(), inner.begin(), inner.end());
    BRep cavity = BRep::from_polylines(faces);
    MINI_CHECK(cavity.shell_count() == 2);
    cavity.m_solids = {{{{0, BRepOrientation::Forward}, {1, BRepOrientation::Reversed}}}};
    MINI_CHECK(cavity.is_solid());
    MINI_CHECK(brep_classify::point(cavity, {0, 0, 0}, 1e-7) == brep_classify::State::Outside);
    MINI_CHECK(brep_classify::point(cavity, {1.5, 0, 0}, 1e-7) == brep_classify::State::Inside);
    MINI_CHECK(brep_classify::point(cavity, {1, 0, 0}, 1e-7) == brep_classify::State::Boundary);
    MINI_CHECK(brep_classify::point(cavity, {3, 0, 0}, 1e-7) == brep_classify::State::Outside);
    cavity.m_solids[0].shells.pop_back();
    MINI_CHECK(brep_classify::point(cavity, {0, 0, 0}, 1e-7) == brep_classify::State::Inside);
}

MINI_TEST("BRep Native Classification", "Rhino Freeform Face Sides") {
    const std::filesystem::path root = std::filesystem::path(__FILE__).parent_path().parent_path();
    const std::vector<BRep> chairs = file_step::read_file_step_breps((root / "tests/fixtures/solidbrep.stp").string());
    MINI_CHECK(chairs.size() == 2);
    nlohmann::ordered_json report = nlohmann::ordered_json::array();
    for (size_t ci = 0; ci < chairs.size(); ++ci) {
        const BRep& chair = chairs[ci];
        int checked = 0;
        for (int fi = 0; fi < chair.face_count(); ++fi) {
            const NurbsSurface& surface = chair.m_surfaces[chair.m_faces[fi].surface_index];
            const std::pair<double, double> du = surface.domain(0);
            const std::pair<double, double> dv = surface.domain(1);
            bool found = false;
            for (int i = 1; i < 8 && !found; ++i)
                for (int j = 1; j < 8 && !found; ++j) {
                    const double u = du.first + (du.second - du.first) * i / 8.0;
                    const double v = dv.first + (dv.second - dv.first) * j / 8.0;
                    if (brep_classify::face_uv(chair, fi, {u, v, 0}, 1e-8) != brep_classify::State::Inside)
                        continue;
                    const Point p = surface.point_at(u, v);
                    Vector normal = surface.normal_at(u, v);
                    if (chair.face_orientation(fi) == BRepOrientation::Reversed)
                        normal = normal * -1.0;
                    if (!normal.normalize_self())
                        continue;
                    const brep_classify::State inside = brep_classify::point(chair, p - normal * 1e-3, 1e-6);
                    const brep_classify::State outside = brep_classify::point(chair, p + normal * 1e-3, 1e-6);
                    report.push_back({{"chair", ci}, {"face", fi}, {"point", {p[0], p[1], p[2]}},
                        {"normal", {normal[0], normal[1], normal[2]}}, {"inside", static_cast<int>(inside)}, {"outside", static_cast<int>(outside)}});
                    std::filesystem::create_directories(root / "build/native-geometry-tests");
                    std::ofstream(root / "build/native-geometry-tests/chair_classification.json") << report.dump(2);
                    MINI_CHECK(inside == brep_classify::State::Inside);
                    MINI_CHECK(outside == brep_classify::State::Outside);
                    ++checked;
                    found = true;
                }
        }
        MINI_CHECK(checked == chair.face_count());
    }
}

MINI_TEST("BRep Native Classification", "Face Boundary Keeps Independent Raw Domains") {
    BRep box = BRep::create_box(2, 2, 2);
    const auto edge = box.wire_edges(box.m_faces[0].wires[0]).front();
    const int pc = box.pcurve_index(edge.index, 0, edge.orientation);
    MINI_CHECK(box.m_curves_2d[pc].set_domain(-7, 13));
    MINI_CHECK(box.m_curves_3d[box.m_edges[edge.index].curve_3d_index].set_domain(32, 35));
    const auto frozen = box.jsondump();
    const auto boundary = brep_face_boundary::extract(box, 0);
    MINI_CHECK(boundary.complete);
    const auto& use = boundary.wires.front().uses.front();
    MINI_CHECK(use.edge.index == edge.index && use.pcurve == pc);
    MINI_CHECK(use.pcurve_domain == std::make_pair(-7.0, 13.0));
    MINI_CHECK(use.has_world_domain && use.world_domain == std::make_pair(32.0, 35.0));
    MINI_CHECK(brep_classify::face_uv(box, 0, Point(.5, .5, 0), 1e-7) == brep_classify::State::Inside);
    MINI_CHECK(box.jsondump() == frozen);
}
MINI_TEST("BRep Native Classification", "Face Boundary Preserves Pole And Seam Uses") {
    BRep sphere = BRep::create_sphere(2);
    for (bool reverse : {false, true}) {
        sphere.m_faces[0].wires[0].orientation = reverse ? BRepOrientation::Reversed : BRepOrientation::Forward;
        const auto boundary = brep_face_boundary::extract(sphere, 0);
        MINI_CHECK(boundary.complete && boundary.wires.size() == 1);
        size_t poles = 0, ordinary = 0;
        int seam = -1, first_pc = -1;
        for (const auto& use : boundary.wires[0].uses) {
            if (use.degenerated) {
                MINI_CHECK(!use.has_world_domain);
                MINI_CHECK(use.first_vertex == use.last_vertex);
                ++poles;
            } else {
                MINI_CHECK(use.has_world_domain);
                if (seam < 0) {
                    seam = use.edge.index;
                    first_pc = use.pcurve;
                }
                else {
                    MINI_CHECK(use.edge.index == seam && use.pcurve != first_pc);
                    const Point a = sphere.m_curves_2d[first_pc].point_at_start();
                    const Point b = sphere.m_curves_2d[use.pcurve].point_at_start();
                    MINI_CHECK(a.distance(b) > 1);
                }
                ++ordinary;
            }
        }
        MINI_CHECK(poles == 2 && ordinary == 2);
        MINI_CHECK(brep_classify::face_uv(sphere, 0, Point(2, 1, 0), 1e-7) == brep_classify::State::Inside);
    }
}
MINI_TEST("BRep Native Classification", "Face Boundary Reports Missing Associated PC") {
    BRep box = BRep::create_box(2, 2, 2);
    const int edge = box.wire_edges(box.m_faces[0].wires[0]).front().index;
    auto& images = box.m_edges[edge].pcurves;
    const int support = box.m_faces[0].surface_index;
    images.erase(std::remove_if(images.begin(), images.end(), [&](const auto& image) {
        return image.surface_index == support;
    }), images.end());
    MINI_CHECK(box.is_valid());
    const auto boundary = brep_face_boundary::extract(box, 0);
    MINI_CHECK(!boundary.complete && boundary.failure == "Boundary use has no pcurve on its support");
    MINI_CHECK(brep_classify::face_uv(box, 0, Point(.5, .5, 0), 1e-7) == brep_classify::State::Unknown);
}
MINI_TEST("BRep Native Classification", "Face Boundary Retains Internal Uses And Checks Vertex Chain") {
    BRep box = BRep::create_box(2, 2, 2);
    auto& wire = box.m_wires[box.m_faces[0].wires[0].index];
    wire.edges.push_back({wire.edges[0].index, BRepOrientation::Internal});
    const auto boundary = brep_face_boundary::extract(box, 0);
    MINI_CHECK(boundary.complete && boundary.wires[0].uses.size() == 5);
    MINI_CHECK(boundary.wires[0].uses.back().edge.orientation == BRepOrientation::Internal);
    MINI_CHECK(brep_classify::face_uv(box, 0, Point(.5, .5, 0), 1e-7) == brep_classify::State::Inside);
    wire.edges.erase(wire.edges.begin());
    MINI_CHECK(!brep_face_boundary::extract(box, 0).complete);
    MINI_CHECK(!box.is_valid());
}

MINI_TEST("BRep Native Classification", "Face Boundary Detects Ambiguous Support Association") {
    BRep box = BRep::create_box(2, 2, 2);
    const auto use = box.wire_edges(box.m_faces[0].wires[0]).front();
    auto& images = box.m_edges[use.index].pcurves;
    const int support = box.m_faces[0].surface_index;
    const int selected = box.pcurve_index(use.index, 0, use.orientation);
    images.push_back({support, selected, -1});
    MINI_CHECK(brep_face_boundary::extract(box, 0).complete);
    const int other = (selected + 1) % static_cast<int>(box.m_curves_2d.size());
    images.push_back({support, other, -1});
    MINI_CHECK(box.is_valid());
    const auto boundary = brep_face_boundary::extract(box, 0);
    MINI_CHECK(!boundary.complete && boundary.failure == "Boundary has ambiguous pcurve associations on one support");
    MINI_CHECK(brep_classify::face_uv(box, 0, Point(.5, .5, 0), 1e-7) == brep_classify::State::Unknown);
}

namespace {
BRep prepared_domain_annulus(double inner) {
    BRep shape;
    auto surface = NurbsSurface::create(false, false, 1, 1, 2, 2,
        {Point(-2,-2,0), Point(-2,2,0), Point(2,-2,0), Point(2,2,0)});
    MINI_CHECK(surface.set_domain(0, -2, 2) && surface.set_domain(1, -2, 2));
    const int support = shape.add_surface(surface);
    std::vector<BRepRef> wires;
    for (double radius : {1.0, inner}) {
        if (radius <= 0) continue;
        const auto circle = Primitives::circle(0, 0, 0, radius);
        const int vertex = shape.add_vertex(circle.point_at_start());
        const int edge = shape.add_edge(shape.add_curve_3d(circle), vertex, vertex);
        shape.add_pcurve(edge, support, shape.add_curve_2d(circle));
        wires.push_back({shape.add_wire({{edge, BRepOrientation::Forward}}), BRepOrientation::Forward});
    }
    shape.add_face(support, wires);
    MINI_CHECK(shape.is_valid());
    return shape;
}
}
MINI_TEST("BRep Native Classification", "Prepared Actual Circle Distinguishes Chord Sagitta") {
    const BRep disk = prepared_domain_annulus(0);
    const auto boundary = brep_face_boundary::extract(disk, 0);
    const auto& circle = disk.m_curves_2d.front();
    const Point a = circle.point_at(0), b = circle.point_at(1.0 / 32);
    const Point arc = circle.point_at(1.0 / 64);
    const Point chord((a[0]+b[0])*.5, (a[1]+b[1])*.5, 0);
    const Point query((arc[0]+chord[0])*.5, (arc[1]+chord[1])*.5, 0);
    const double radius = std::hypot(query[0], query[1]);
    MINI_CHECK(radius < 1 && 1-radius < .001);
    const Vector normal(a[0]+b[0], a[1]+b[1], 0);
    MINI_CHECK(query[0]*normal[0]+query[1]*normal[1] > chord[0]*normal[0]+chord[1]*normal[1]);
    MINI_CHECK(brep_classify::face_uv(boundary, query, .001) == brep_classify::State::Boundary);
    MINI_CHECK(brep_classify::face_uv(disk, 0, query, .001) == brep_classify::face_uv(boundary, query, .001));
}
MINI_TEST("BRep Native Classification", "Prepared Annulus Keeps Holes Raw Domains And Reversal") {
    BRep annulus = prepared_domain_annulus(.5);
    for (auto& pc : annulus.m_curves_2d)
        MINI_CHECK(pc.set_domain(-7, 13));
    for (bool reversed : {false, true}) {
        for (auto& wire : annulus.m_faces[0].wires)
            wire.orientation = reversed ? BRepOrientation::Reversed : BRepOrientation::Forward;
        const auto boundary = brep_face_boundary::extract(annulus, 0);
        const std::array<Point, 4> probes = {Point(0,0,0), Point(.75,0,0), Point(1.5,0,0), Point(1,0,0)};
        const std::array<brep_classify::State, 4> expected = {brep_classify::State::Outside,
            brep_classify::State::Inside, brep_classify::State::Outside, brep_classify::State::Boundary};
        for (size_t i = 0; i < probes.size(); ++i) {
            MINI_CHECK(brep_classify::face_uv(boundary, probes[i], 1e-5) == expected[i]);
            MINI_CHECK(brep_classify::face_uv(annulus, 0, probes[i], 1e-5) == expected[i]);
        }
    }
    const BRep narrow = prepared_domain_annulus(1 - 6e-5);
    const auto boundary = brep_face_boundary::extract(narrow, 0);
    MINI_CHECK(brep_classify::face_uv(boundary, Point(1-3e-5,0,0), 1e-5) == brep_classify::State::Inside);
}

MINI_TEST("BRep Native Classification", "Parity Ray Stays Clear Of Corner Gaps") {
    const double corner = 0.75, last_bits = std::nextafter(corner, 1.0);
    const NurbsCurve first = NurbsCurve::create(false, 1, {Point(1, 0, 0), Point(1, corner, 0)});
    const NurbsCurve second = NurbsCurve::create(false, 1, {Point(1, last_bits, 0), Point(1, 2, 0)});
    const double height = brep_classify::clear_ray_height({&first, &second}, corner, 1e-10);
    MINI_CHECK(height != corner && std::abs(height - corner) < 1e-10);
    MINI_CHECK(brep_classify::clear_ray_height({&first, &second}, 0.5, 1e-10) == 0.5);
}

MINI_TEST("BRep Native Classification", "Prepared Solid Answers As The Model") {
    const std::vector<BRep> shapes = {BRep::create_box(2, 2, 2), BRep::create_sphere(1.0), BRep::create_torus(1.0, 0.25),
        BRep::create_block_with_hole(2, 2, 1, 0.3).transformed(Xform::rotation(Vector(1, 2, 3), 0.7))};
    const std::vector<Point> queries = {{0, 0, 0}, {0.9, 0.1, 0.2}, {1, 0, 0}, {1, 1, 0}, {0.31, -0.47, 0.13}, {3, 0.2, -0.1}, {1.25, 0, 0}};
    for (const BRep& shape : shapes) {
        const brep_classify::Solid solid(shape), asked(shape, false);
        MINI_CHECK(solid.valid && solid.solid && static_cast<int>(solid.faces.size()) == shape.face_count());
        MINI_CHECK(asked.valid && asked.solid && !asked.faces[0].boundary.owner && solid.faces[0].boundary.complete);
        for (const Point& query : queries) {
            MINI_CHECK(brep_classify::point(solid, query, 1e-7) == brep_classify::point(shape, query, 1e-7));
            const auto prepared = brep_classify::ray(solid, query, Vector(0.319, -0.271, 1.0), 1e-7);
            const auto direct = brep_classify::ray(shape, query, Vector(0.319, -0.271, 1.0), 1e-7);
            MINI_CHECK(prepared.complete == direct.complete && prepared.hits.size() == direct.hits.size());
            for (size_t i = 0; i < prepared.hits.size() && i < direct.hits.size(); ++i)
                MINI_CHECK(prepared.hits[i].face == direct.hits[i].face && prepared.hits[i].distance == direct.hits[i].distance
                    && prepared.hits[i].crossing == direct.hits[i].crossing && prepared.hits[i].trim == direct.hits[i].trim);
        }
        for (int face = 0; face < shape.face_count(); ++face) {
            const NurbsSurface& surface = shape.m_surfaces[shape.m_faces[face].surface_index];
            for (double s : {-0.1, 0.0, 0.3, 0.5, 1.0})
                for (double t : {0.25, 0.5, 1.2}) {
                    const Point uv(std::lerp(surface.domain(0).first, surface.domain(0).second, s),
                        std::lerp(surface.domain(1).first, surface.domain(1).second, t), 0);
                    MINI_CHECK(brep_classify::face_uv(solid, face, uv, 1e-7) == brep_classify::face_uv(shape, face, uv, 1e-7));
                    MINI_CHECK(brep_classify::face_uv(asked, face, uv, 1e-7) == brep_classify::face_uv(shape, face, uv, 1e-7));
                }
        }
    }
    BRep open = shapes[0];
    open.m_shells[0].faces.pop_back();
    const brep_classify::Solid sheet(open);
    MINI_CHECK(sheet.valid && !sheet.solid);
    MINI_CHECK(brep_classify::ray(sheet, {0, 0, 0}, Vector(1, 0, 0), 1e-7).hits.size() == 1);
    BRep broken = shapes[0];
    broken.m_curves_2d[0].m_cv[0] = std::numeric_limits<double>::quiet_NaN();
    const brep_classify::Solid invalid(broken);
    int rejected = 0;
    for (int query = 0; query < 4; ++query)
        try {
            if (query == 0) static_cast<void>(brep_classify::point(sheet, {0, 0, 0}, 1e-7));
            if (query == 1) static_cast<void>(brep_classify::point(invalid, {0, 0, 0}, 1e-7));
            if (query == 2) static_cast<void>(brep_classify::ray(invalid, {0, 0, 0}, Vector(1, 0, 0), 1e-7));
            if (query == 3) static_cast<void>(brep_classify::face_uv(invalid, 0, {0.5, 0.5, 0}, 1e-7));
        } catch (const std::invalid_argument&) {
            ++rejected;
        }
    MINI_CHECK(!invalid.valid && rejected == 4);
}

MINI_TEST("BRep Native Classification", "One Witness Decides Faces Joined Off The Other Solid") {
    const BRep outer = BRep::create_box(4, 4, 4);
    const BRep inner = BRep::create_sphere(1.0).transformed(Xform::translation(0.3, -0.2, 0.1));
    const double ball = 4.0 * Tolerance::PI / 3.0;
    MINI_CHECK(std::abs(outer.boolean_union(inner).volume_exact() - 64.0) < 1e-6);
    MINI_CHECK(std::abs(outer.boolean_intersection(inner).volume_exact() - ball) < 1e-6);
    const BRep hollow = outer.boolean_difference(inner);
    MINI_CHECK(hollow.face_count() == 7 && hollow.shell_count() == 2 && std::abs(hollow.volume_exact() - (64.0 - ball)) < 1e-6);
    MINI_CHECK(inner.boolean_difference(outer).face_count() == 0);
    // A face lying in a face of the other solid is decided by itself, not by its neighbours.
    const BRep stacked = BRep::create_box(2, 2, 2).transformed(Xform::translation(0, 0, 3));
    const BRep tower = outer.boolean_union(stacked);
    MINI_CHECK(tower.is_solid() && std::abs(tower.volume_exact() - 72.0) < 1e-6);
    MINI_CHECK(outer.boolean_intersection(stacked).face_count() == 0);
    const BRep sunk = BRep::create_box(2, 2, 2).transformed(Xform::translation(0, 0, 1));
    MINI_CHECK(std::abs(outer.boolean_union(sunk).volume_exact() - 64.0) < 1e-6);
    MINI_CHECK(std::abs(outer.boolean_difference(sunk).volume_exact() - 56.0) < 1e-6);
}

MINI_TEST("BRep Native Classification", "Faces Left Active Answer As Their Shell Alone") {
    const BRep hollow = BRep::create_box(4, 4, 4).boolean_difference(BRep::create_sphere(1.0).transformed(Xform::translation(0.3, -0.2, 0.1)));
    MINI_CHECK(hollow.shell_count() == 2);
    brep_classify::Solid solid(hollow);
    const std::vector<Point> queries = {{0.3, -0.2, 0.1}, {1.3, -0.2, 0.1}, {1.8, 0, 0}, {2, 0.5, 0.5}, {5, 0, 0}, {0.3, 0.8, 0.1 + 1e-8}};
    int centre = 0, beside = 0;
    for (int shell = 0; shell < 2; ++shell) {
        BRep alone = hollow;
        alone.m_solids.clear();
        alone.add_solid({{shell, BRepOrientation::Forward}});
        for (brep_classify::Face& face : solid.faces)
            face.active = false;
        for (const BRepRef& face : hollow.m_shells[shell].faces)
            solid.faces[face.index].active = true;
        for (const Point& query : queries)
            MINI_CHECK(brep_classify::point(solid, query, 1e-7) == brep_classify::point(alone, query, 1e-7));
        centre += brep_classify::point(solid, queries[0], 1e-7) == brep_classify::State::Inside;
        beside += brep_classify::point(solid, queries[2], 1e-7) == brep_classify::State::Inside;
    }
    MINI_CHECK(centre == 2 && beside == 1);
}

MINI_TEST("BRep Native Classification", "Knot Inserted In Place As With Fresh Arrays") {
    // Boehm's insertion written out on copies of the arrays: the curve's own insertion gives these bits.
    const auto reference = [](NurbsCurve curve, double value, int times) {
        for (int pass = 0; pass < times; ++pass) {
            const int p = curve.degree(), stride = curve.m_cv_stride;
            const int k = nurbsknot::find_span(curve.order(), curve.cv_count(), curve.m_nurbsknot, value) + p;
            std::vector<double> full = curve.m_nurbsknot;
            full.insert(full.begin(), full.front());
            full.push_back(full.back());
            std::vector<double> cv(curve.m_cv.begin(), curve.m_cv.begin() + (k - p + 1) * stride);
            for (int i = k - p + 1; i <= k; ++i) {
                const double span = full[i + p] - full[i], alpha = span != 0.0 ? (value - full[i]) / span : 0.0;
                for (int d = 0; d < stride; ++d) {
                    const double before = curve.m_cv[(i - 1) * stride + d], here = curve.m_cv[i * stride + d];
                    cv.push_back(std::isfinite(here) && here != 0.0 && before == here ? here : (1.0 - alpha) * before + alpha * here);
                }
            }
            cv.insert(cv.end(), curve.m_cv.begin() + k * stride, curve.m_cv.end());
            curve.m_cv = cv;
            curve.m_cv_count++;
            curve.m_nurbsknot.insert(curve.m_nurbsknot.begin() + k, value);
        }
        return curve;
    };
    NurbsCurve plain = NurbsCurve::create(false, 3, {{0, 0, 0}, {1, 2, 0.5}, {2, -1, 1}, {4, 0.5, 0}, {5, 3, 2}, {7, 1, 1}, {8, 0, 3}});
    NurbsCurve weighted = Primitives::circle(0.3, -0.2, 0.1, 1.7);
    for (const NurbsCurve* source : {&plain, &weighted})
        for (double share : {0.137, 0.41, 0.86})
            for (int times = 1; times <= source->degree(); ++times) {
                const double value = std::lerp(source->domain_start(), source->domain_end(), share);
                NurbsCurve inserted = *source;
                MINI_CHECK(inserted.insert_nurbsknot(value, times));
                const NurbsCurve expected = reference(*source, value, times);
                MINI_CHECK(inserted.cv_count() == source->cv_count() + times && inserted.is_valid());
                MINI_CHECK(inserted.m_cv == expected.m_cv && inserted.m_nurbsknot == expected.m_nurbsknot);
                MINI_CHECK(inserted.insert_nurbsknot(value, times) && inserted.m_cv == expected.m_cv);
            }
}
