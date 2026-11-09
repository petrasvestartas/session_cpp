#include "brep_classify.h"
#include "nurbs_bounds.h"
#include "brep_face_boundary.h"
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

MINI_TEST("BRep Native Classification", "Original Pooled Circle Uses Retain Quarter Ranges") {
    BRep shape;
    auto support=NurbsSurface::create(false,false,1,1,2,2,
        {Point(-2,-2,0),Point(-2,2,0),Point(2,-2,0),Point(2,2,0)});
    MINI_CHECK(support.set_domain(0,-2,2) && support.set_domain(1,-2,2));
    const int surface=shape.add_surface(support);
    auto circle=Primitives::circle(0,0,0,1);
    MINI_CHECK(circle.set_domain(-7,13));
    const int world=shape.add_curve_3d(circle), pc=shape.add_curve_2d(circle);
    const std::array<double,5> cuts{-7,-2,3,8,13};
    std::array<int,4> vertices;
    for (size_t i=0;i<vertices.size();++i)
        vertices[i]=shape.add_vertex(circle.point_at(cuts[i]));
    std::vector<BRepRef> uses;
    for (size_t i=0;i<vertices.size();++i) {
        const int edge=shape.add_edge(world,vertices[i],vertices[(i+1)%4]);
        shape.m_edges[edge].world_range=BRepCurveUseRange{cuts[i],cuts[i+1]};
        shape.add_pcurve(edge,surface,pc);
        shape.m_edges[edge].pcurves.back().primary_range=BRepCurveUseRange{cuts[i],cuts[i+1]};
        uses.push_back({edge,BRepOrientation::Forward});
    }
    shape.add_face(surface,{{shape.add_wire(uses),BRepOrientation::Forward}});
    MINI_CHECK(shape.is_valid());
    for (bool reversed : {false,true}) {
        shape.m_faces[0].wires[0].orientation=reversed?BRepOrientation::Reversed:BRepOrientation::Forward;
        const auto frozen=shape.jsondump();
        const auto boundary=brep_face_boundary::extract(shape,0);
        MINI_CHECK(boundary.complete && boundary.wires[0].uses.size()==4);
        MINI_CHECK(brep_classify::face_uv(boundary,Point(0,0,0),1e-7)==brep_classify::State::Inside);
        MINI_CHECK(brep_classify::face_uv(boundary,Point(1.1,0,0),1e-7)==brep_classify::State::Outside);
        MINI_CHECK(brep_classify::face_uv(boundary,Point(1,0,0),1e-7)==brep_classify::State::Boundary);
        MINI_CHECK(shape.m_curves_3d.size()==1 && shape.m_curves_2d.size()==1);
        MINI_CHECK(shape.jsondump()==frozen);
    }
}

MINI_TEST("BRep Native Classification", "Compressed Affine Ranged Boundary Keeps Whole UV Winding") {
    const double tolerance=std::ldexp(1.,-20),height=tolerance/4;
    BRep owner;
    const auto support=NurbsSurface::create(false,false,1,1,2,2,
        {Point(0,0,0),Point(0,height,0),Point(1,0,0),Point(1,height,0)});
    const int surface=owner.add_surface(support);
    const std::array<Point,6> uv{Point(0,0,0),Point(1,0,0),Point(1,.25,0),
        Point(1,.75,0),Point(1,1,0),Point(0,1,0)};
    std::array<int,6> vertices;
    for (size_t i=0;i<vertices.size();++i)
        vertices[i]=i==3?vertices[2]:owner.add_vertex(i==2?Point(1,height*.5,0):support.point_at(uv[i][0],uv[i][1]));
    auto right_world=NurbsCurve::create(false,1,{Point(1,0,0),Point(1,height,0)});
    auto right_pc=NurbsCurve::create(false,1,{Point(1,0,0),Point(1,1,0)});
    MINI_CHECK(right_world.set_domain(0,1) && right_pc.set_domain(0,1));
    const int right=owner.add_curve_3d(right_world),right_image=owner.add_curve_2d(right_pc);
    std::vector<BRepRef> uses;
    for (size_t i=0;i<uv.size();++i) {
        const size_t j=(i+1)%uv.size();
        int world=right,pc=right_image;
        if (i<1 || i>3) {
            world=owner.add_curve_3d(NurbsCurve::create(false,1,
                {support.point_at(uv[i][0],uv[i][1]),support.point_at(uv[j][0],uv[j][1])}));
            pc=owner.add_curve_2d(NurbsCurve::create(false,1,{uv[i],uv[j]}));
        }
        const int edge=owner.add_edge(world,vertices[i],vertices[j]);
        owner.add_pcurve(edge,surface,pc);
        if (i>=1 && i<=3) {
            owner.m_edges[edge].world_range=BRepCurveUseRange{uv[i][1],uv[j][1]};
            owner.m_edges[edge].pcurves.back().primary_range=BRepCurveUseRange{uv[i][1],uv[j][1]};
        }
        uses.push_back({edge,BRepOrientation::Forward});
    }
    owner.add_face(surface,{{owner.add_wire(uses),BRepOrientation::Forward}});
    MINI_CHECK(owner.is_valid());
    const double uv_tol=tolerance/nurbs_bounds::surface_metric(support);
    for (bool reversed : {false,true}) {
        owner.m_faces[0].wires[0].orientation=reversed?BRepOrientation::Reversed:BRepOrientation::Forward;
        const auto frozen=owner.jsondump();
        MINI_CHECK(brep_classify::face_uv(owner,0,Point(.5,.5,0),uv_tol)==brep_classify::State::Inside);
        MINI_CHECK(brep_classify::face_uv(owner,0,Point(1,.5,0),uv_tol)==brep_classify::State::Boundary);
        MINI_CHECK(owner.jsondump()==frozen);
    }
}

MINI_TEST("BRep Native Classification", "Ranged World Edge Excludes Unused Pooled Continuation") {
    BRep owner=BRep::create_box(2,2,2);
    const auto original=owner.m_curves_3d[owner.m_edges[0].curve_3d_index];
    const Point first=original.point_at_start(),last=original.point_at_end();
    const Vector direction=last-first;
    const Point query=first+direction*1.5;
    MINI_CHECK(brep_classify::point(owner,query,1e-7)==brep_classify::State::Outside);
    auto pooled=NurbsCurve::create(false,1,{first-direction,last+direction});
    MINI_CHECK(pooled.set_domain(-1,2));
    const int world=owner.add_curve_3d(pooled);
    owner.m_edges[0].curve_3d_index=world;
    owner.m_edges[0].world_range=BRepCurveUseRange{0,1};
    for (auto& association : owner.m_edges[0].pcurves) {
        for (bool secondary : {false,true}) {
            int& index=secondary?association.curve_2d_index_2:association.curve_2d_index;
            if (index<0)
                continue;
            const auto& source=owner.m_curves_2d[index];
            const Point a=source.point_at_start(),b=source.point_at_end();
            auto pc=NurbsCurve::create(false,1,{a-(b-a),b+(b-a)});
            MINI_CHECK(pc.set_domain(-1,2));
            index=owner.add_curve_2d(pc);
            auto& range=secondary?association.secondary_range:association.primary_range;
            range=BRepCurveUseRange{0,1};
        }
        const auto original_support=owner.m_surfaces[association.surface_index];
        const auto u=original_support.domain(0),v=original_support.domain(1);
        const double a=u.first-(u.second-u.first),b=u.second+(u.second-u.first);
        const double c=v.first-(v.second-v.first),d=v.second+(v.second-v.first);
        auto extended=NurbsSurface::create(false,false,1,1,2,2,
            {original_support.point_at(a,c),original_support.point_at(a,d),
             original_support.point_at(b,c),original_support.point_at(b,d)});
        MINI_CHECK(extended.set_domain(0,a,b) && extended.set_domain(1,c,d));
        owner.m_surfaces[association.surface_index]=extended;
    }
    MINI_CHECK(owner.is_valid() && owner.is_solid());
    MINI_CHECK(pooled.point_at(1.5).distance(query)<1e-12);
    const auto frozen=owner.jsondump();
    MINI_CHECK(brep_classify::point(owner,query,1e-7)==brep_classify::State::Outside);
    MINI_CHECK(brep_classify::point(owner,first+direction*.5,1e-7)==brep_classify::State::Boundary);
    MINI_CHECK(owner.jsondump()==frozen);
}
