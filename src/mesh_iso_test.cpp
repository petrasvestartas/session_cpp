#include "mini_test.h"
#include "mesh_iso.h"
#include "boundingbox.h"
#include "point.h"
#include "tolerance.h"

#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("MeshIso", "Eval Gyroid") {
    MINI_CHECK(TOLERANCE.is_close(MeshIso::eval(TpmsType::GYROID, Tolerance::PI / 2.0, 0.0, 0.0, 1.0), 1.0));
    MINI_CHECK(TOLERANCE.is_close(MeshIso::eval(TpmsType::SCHWARZ_P, 0.0, 0.0, 0.0, 1.0), 3.0));
}

MINI_TEST("MeshIso", "Eval SchwarzP") {
    MINI_CHECK(TOLERANCE.is_close(MeshIso::eval(TpmsType::SCHWARZ_P, Tolerance::PI, Tolerance::PI, Tolerance::PI, 1.0), -3.0));
}

MINI_TEST("MeshIso", "Eval Diamond") {
    MINI_CHECK(TOLERANCE.is_close(MeshIso::eval(TpmsType::DIAMOND, 0.0, 0.0, 0.0, 1.0), 0.0));
}

MINI_TEST("MeshIso", "From Tpms Gyroid Solid") {
    BoundingBox box = BoundingBox::from_points({Point(0.0, 0.0, 0.0), Point(1.0, 1.0, 1.0)});
    Mesh m = MeshIso::from_tpms(TpmsType::GYROID, box, 10, 10, 10, 0.0, 1.0, TpmsMode::SOLID);
    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() > 0);
    MINI_CHECK(m.number_of_faces() > 0);
}

MINI_TEST("MeshIso", "From Tpms Diamond Sheet") {
    BoundingBox box = BoundingBox::from_points({Point(0.0, 0.0, 0.0), Point(1.0, 1.0, 1.0)});
    Mesh m = MeshIso::from_tpms(TpmsType::DIAMOND, box, 10, 10, 10, 0.0, 1.0, TpmsMode::SHEET, 0.1);
    MINI_CHECK(m.is_valid());
}

MINI_TEST("MeshIso", "From Tpms Neovius Shell") {
    BoundingBox box = BoundingBox::from_points({Point(0.0, 0.0, 0.0), Point(1.0, 1.0, 1.0)});
    Mesh m = MeshIso::from_tpms(TpmsType::NEOVIUS, box, 10, 10, 10, 0.0, 1.0, TpmsMode::SHELL, 0.1);
    MINI_CHECK(m.is_valid());
}

MINI_TEST("MeshIso", "SDF Sphere") {
    MINI_CHECK(TOLERANCE.is_close(MeshIso::sdf_sphere(0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0), 0.0));
    MINI_CHECK(TOLERANCE.is_close(MeshIso::sdf_sphere(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0), -1.0));
    MINI_CHECK(TOLERANCE.is_close(MeshIso::sdf_sphere(0.0, 0.0, 0.0, 1.0, 2.0, 0.0, 0.0), 1.0));
}

MINI_TEST("MeshIso", "Smooth Union") {
    MINI_CHECK(MeshIso::smooth_union(-1.0, 1.0, 8.0) < 0.0);
    MINI_CHECK(MeshIso::smooth_union(1.0, 1.0, 8.0) < 1.0);
}

MINI_TEST("MeshIso", "From Function") {
    BoundingBox box = BoundingBox::from_points({Point(-2.0, -2.0, -2.0), Point(2.0, 2.0, 2.0)});
    auto fn = [](double x, double y, double z) {
        return MeshIso::sdf_sphere(0.0, 0.0, 0.0, 1.0, x, y, z);
    };
    Mesh m = MeshIso::from_function(fn, box, 10, 10, 10, 0.0);
    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() > 0);
}

MINI_TEST("MeshIso", "All Tpms Shells") {
    BoundingBox box = BoundingBox::from_points({Point(0.0, 0.0, 0.0), Point(1.0, 1.0, 1.0)});
    TpmsType types[] = {
        TpmsType::GYROID, TpmsType::SCHWARZ_P, TpmsType::DIAMOND,
        TpmsType::NEOVIUS, TpmsType::IWP, TpmsType::LIDINOID,
        TpmsType::FISCHER_KOCH_S, TpmsType::FRD, TpmsType::PMY,
    };
    for (int i = 0; i < 9; ++i) {
        Mesh m = MeshIso::from_tpms(types[i], box, 10, 10, 10, 0.0, 1.0, TpmsMode::SHELL, 0.1);
        MINI_CHECK(m.is_valid());
        MINI_CHECK(m.number_of_vertices() > 0);
    }
}

MINI_TEST("MeshIso", "SDF Box") {
    BoundingBox box = BoundingBox::from_points({Point(-2.0, -2.0, -2.0), Point(2.0, 2.0, 2.0)});
    Mesh m = MeshIso::from_function([](double x, double y, double z) {
        return MeshIso::sdf_box(0.0, 0.0, 0.0, 1.0, 0.7, 1.3, x, y, z);
    }, box, 10, 10, 10);
    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() > 0);
}

MINI_TEST("MeshIso", "SDF Torus") {
    BoundingBox box = BoundingBox::from_points({Point(-2.0, -2.0, -2.0), Point(2.0, 2.0, 2.0)});
    Mesh m = MeshIso::from_function([](double x, double y, double z) {
        return MeshIso::sdf_torus(0.0, 0.0, 0.0, 1.1, 0.4, x, y, z);
    }, box, 10, 10, 10);
    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() > 0);
}

MINI_TEST("MeshIso", "SDF Capsule") {
    BoundingBox box = BoundingBox::from_points({Point(-2.0, -2.0, -2.0), Point(2.0, 2.0, 2.0)});
    Mesh m = MeshIso::from_function([](double x, double y, double z) {
        return MeshIso::sdf_capsule(Point(0.0, -1.0, 0.0), Point(0.0, 1.0, 0.0), 0.5, x, y, z);
    }, box, 10, 10, 10);
    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() > 0);
}

MINI_TEST("MeshIso", "Smooth Subtract") {
    BoundingBox box = BoundingBox::from_points({Point(-2.0, -2.0, -2.0), Point(2.0, 2.0, 2.0)});
    Mesh m = MeshIso::from_function([](double x, double y, double z) {
        double a = MeshIso::sdf_sphere(0.0, 0.0, 0.0, 1.2, x, y, z);
        double b = MeshIso::sdf_box(0.0, 0.0, 0.0, 0.8, 0.8, 0.8, x, y, z);
        return MeshIso::smooth_subtract(a, b, 8.0);
    }, box, 15, 15, 15);
    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() > 0);
}

MINI_TEST("MeshIso", "Smooth Intersect") {
    BoundingBox box = BoundingBox::from_points({Point(-2.0, -2.0, -2.0), Point(2.0, 2.0, 2.0)});
    Mesh m = MeshIso::from_function([](double x, double y, double z) {
        double a = MeshIso::sdf_sphere(0.0, 0.0, 0.0, 1.4, x, y, z);
        double b = MeshIso::sdf_box(0.0, 0.0, 0.0, 1.1, 1.1, 1.1, x, y, z);
        return MeshIso::smooth_intersect(a, b, 8.0);
    }, box, 15, 15, 15);
    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() > 0);
}

MINI_TEST("MeshIso", "Gyroid Sphere Shell") {
    BoundingBox box = BoundingBox::from_points({Point(-1.6, -1.6, -1.6), Point(1.6, 1.6, 1.6)});
    Mesh m = MeshIso::from_function([](double x, double y, double z) {
        double tpms = MeshIso::eval(TpmsType::GYROID, x, y, z, 1.0);
        double shell = std::abs(MeshIso::sdf_sphere(0.0, 0.0, 0.0, 1.3, x, y, z)) - 0.08;
        return MeshIso::smooth_union(tpms * 0.3, shell, 8.0);
    }, box, 10, 10, 10);
    MINI_CHECK(m.is_valid());
    MINI_CHECK(m.number_of_vertices() > 0);
}

} // namespace session_cpp
