#include "brep_classify.h"
#include "file_step.h"
#include "mini_test.h"
#include <array>
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
