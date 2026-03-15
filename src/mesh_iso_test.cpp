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

} // namespace session_cpp
