#include "mini_test.h"
#include "vector.h"
#include "point.h"
#include "tolerance.h"

#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("Vector", "constructor") {
    // uncomment #include "vector.h"
    // uncomment #include "point.h"

    // Constructor
    Vector v(1.0, 2.0, 3.0);
    Point p0(1.0, 2.0, 3.0);
    Point p1(2.0, 4.0, 6.0);
    Vector v_2p = Vector::from_points(p0, p1);

    // Setters
    v[0] = 10.0;
    v[1] = 20.0;
    v[2] = 30.0;

    // Getters
    double x = v[0];
    double y = v[1];
    double z = v[2];

    // Minimal and full string representation
    std::string vstr = v.str();
    std::string vrepr = v.repr();

    // Copy (duplicate everything but guid)
    Vector vcopy = v.duplicate();
    Vector vother(1.0, 2.0, 3.0);

    // No-copy operators
    Vector vmult = v.duplicate();
    vmult *= 2.0;
    Vector vdiv = v.duplicate();
    vdiv /= 2.0;
    Vector vadd = v.duplicate();
    vadd += Vector(1.0, 1.0, 1.0);
    Vector vsub = v.duplicate();
    vsub -= Vector(1.0, 1.0, 1.0);

    // Copy operators
    Vector result_mul = v * 2.0;
    Vector result_div = v / 2.0;
    Vector result_add = v + Vector(1.0, 1.0, 1.0);
    Vector result_dif = v - Vector(1.0, 1.0, 1.0);

    // Static axis constructors
    Vector vx = Vector::x_axis();
    Vector vy = Vector::y_axis();
    Vector vz = Vector::z_axis();
    Vector vzero = Vector::zero();

    MINI_CHECK(v.name == "my_vector" && v[0] == 10.0 && v[1] == 20.0 && v[2] == 30.0 && !v.guid.empty());
    MINI_CHECK(x == 10.0 && y == 20.0 && z == 30.0);
    MINI_CHECK(v_2p[0] == 1.0 && v_2p[1] == 2.0 && v_2p[2] == 3.0);
    MINI_CHECK(vstr == "10.000000, 20.000000, 30.000000");
    MINI_CHECK(vrepr == "Vector(my_vector, 10.000000, 20.000000, 30.000000, 37.416574)");
    MINI_CHECK(vcopy == v && vcopy.guid != v.guid);
    MINI_CHECK(vother != v);
    MINI_CHECK(vmult[0] == 20.0 && vmult[1] == 40.0 && vmult[2] == 60.0);
    MINI_CHECK(vdiv[0] == 5.0 && vdiv[1] == 10.0 && vdiv[2] == 15.0);
    MINI_CHECK(vadd[0] == 11.0 && vadd[1] == 21.0 && vadd[2] == 31.0);
    MINI_CHECK(vsub[0] == 9.0 && vsub[1] == 19.0 && vsub[2] == 29.0);
    MINI_CHECK(result_mul[0] == 20.0 && result_mul[1] == 40.0 && result_mul[2] == 60.0);
    MINI_CHECK(result_div[0] == 5.0 && result_div[1] == 10.0 && result_div[2] == 15.0);
    MINI_CHECK(result_add[0] == 11.0 && result_add[1] == 21.0 && result_add[2] == 31.0);
    MINI_CHECK(result_dif[0] == 9.0 && result_dif[1] == 19.0 && result_dif[2] == 29.0);
    MINI_CHECK(vx[0] == 1.0 && vx[1] == 0.0 && vx[2] == 0.0);
    MINI_CHECK(vy[0] == 0.0 && vy[1] == 1.0 && vy[2] == 0.0);
    MINI_CHECK(vz[0] == 0.0 && vz[1] == 0.0 && vz[2] == 1.0);
    MINI_CHECK(vzero[0] == 0.0 && vzero[1] == 0.0 && vzero[2] == 0.0);
}

MINI_TEST("Vector", "magnitude") {
    // uncomment #include "vector.h"

    Vector v(3.0, 4.0, 0.0);
    double length = v.magnitude();
    double len_squared = v.magnitude_squared();

    MINI_CHECK(length == 5.0);
    MINI_CHECK(len_squared == 25.0);
}

MINI_TEST("Vector", "normalize") {
    // uncomment #include "vector.h"
    // uncomment #include "tolerance.h"

    Vector v0(3.0, 4.0, 0.0);
    v0.normalize_self();

    Vector v1(3.0, 4.0, 0.0);
    Vector v2 = v1.normalize();

    MINI_CHECK(TOLERANCE.is_close(v0.magnitude(), 1.0));
    MINI_CHECK(TOLERANCE.is_close(v2.magnitude(), 1.0));
}

MINI_TEST("Vector", "reverse") {
    // uncomment #include "vector.h"

    Vector v(1.0, -2.0, 3.0);
    v.reverse();

    MINI_CHECK(v[0] == -1.0 && v[1] == 2.0 && v[2] == -3.0);
}

MINI_TEST("Vector", "dot_product") {
    // uncomment #include "vector.h"

    // Orthogonality and parallelism via dot product
    // Perpendicular vectors are close to 0.0
    // Parallel vectors are close to 1.0
    Vector v1(1.0, 0.0, 0.0);
    Vector v2(0.0, 1.0, 0.0);
    Vector v3(1.0, 0.0, 0.0);
    double dot_perp = v1.dot(v2);
    double dot_paral = v1.dot(v3);

    // Projection of a onto b
    // Scalar projection:
    // (a . b) / ||b|| (here ||b||=1, so just a_x = 3.0)
    // Projection coefficient:
    // (a . b) / ||b||^2 = 6/4 = 1.5 (how many b2's fit in projection)
    Vector a(3.0, 4.0, 0.0);
    Vector b(1.0, 0.0, 0.0);
    Vector b2(2.0, 0.0, 0.0);
    double proj_scalar = a.dot(b) / std::sqrt(b[0]*b[0] + b[1]*b[1] + b[2]*b[2]);
    double proj_coeff = a.dot(b2) / (b2[0]*b2[0] + b2[1]*b2[1] + b2[2]*b2[2]);

    MINI_CHECK(TOLERANCE.is_close(dot_perp, 0.0));
    MINI_CHECK(TOLERANCE.is_close(dot_paral, 1.0));
    MINI_CHECK(TOLERANCE.is_close(proj_scalar, 3.0));
    MINI_CHECK(TOLERANCE.is_close(proj_coeff, 1.5));
}

MINI_TEST("Vector", "cross_product") {
    // uncomment #include "vector.h"

    // Get normal
    Vector v1(1.0, 0.0, 0.0);
    Vector v2(0.0, 1.0, 0.0);
    Vector vn = v1.cross(v2);

    // Perpendicular, area = 3*4=12
    Vector a(3.0, 0.0, 0.0);
    Vector b(0.0, 4.0, 0.0);
    Vector cross = a.cross(b);
    double area = std::sqrt(cross[0]*cross[0] + cross[1]*cross[1] + cross[2]*cross[2]);

    MINI_CHECK(vn[0] == 0.0 && vn[1] == 0.0 && vn[2] == 1.0);
    MINI_CHECK(TOLERANCE.is_close(area, 12.0));
}

MINI_TEST("Vector", "angle") {
    // uncomment #include "vector.h"

    // angle(): Angle between two vectors (degrees)
    Vector v1(1.0, 0.0, 0.0);  // x-axis
    Vector v2(0.0, 1.0, 0.0);  // y-axis
    Vector v3(1.0, 1.0, 0.0);  // 45 deg from x-axis

    double angle_90 = v1.angle(v2, false);  // v1 to v2: 90 deg
    double angle_45 = v1.angle(v3, false);  // v1 to v3: 45 deg

    // angle_between_vector_xy_components(): Angle of vector's XY projection from +X axis (atan2)
    Vector v_30(std::sqrt(3.0), 1.0, 0.0);  // 30 deg from x-axis
    Vector v_60(1.0, std::sqrt(3.0), 0.0);  // 60 deg from x-axis
    Vector v_neg(-1.0, 1.0, 0.0);           // 135 deg from x-axis

    double xy_angle_30 = Vector::angle_between_vector_xy_components(v_30);
    double xy_angle_60 = Vector::angle_between_vector_xy_components(v_60);
    double xy_angle_135 = Vector::angle_between_vector_xy_components(v_neg);

    // coordinate_direction_3angles(): Angles (alpha, beta, gamma) to x, y, z axes
    Vector v_dir(35.4, 35.4, 86.6);
    auto abg = v_dir.coordinate_direction_3angles(true);

    // coordinate_direction_2angles(): Spherical angles (theta azimuth, phi elevation)
    Vector v_sph(1.0, 1.0, std::sqrt(2.0));
    auto pt = v_sph.coordinate_direction_2angles(true);

    MINI_CHECK(TOLERANCE.is_close(angle_90, 90.0));
    MINI_CHECK(TOLERANCE.is_close(angle_45, 45.0));
    MINI_CHECK(TOLERANCE.is_close(xy_angle_30, 30.0));
    MINI_CHECK(TOLERANCE.is_close(xy_angle_60, 60.0));
    MINI_CHECK(TOLERANCE.is_close(xy_angle_135, 135.0));
    MINI_CHECK(TOLERANCE.is_close(std::get<0>(abg), 69.2742));
    MINI_CHECK(TOLERANCE.is_close(std::get<1>(abg), 69.2742));
    MINI_CHECK(TOLERANCE.is_close(std::get<2>(abg), 30.032058));
    MINI_CHECK(TOLERANCE.is_close(std::get<0>(pt), 45.0));
    MINI_CHECK(TOLERANCE.is_close(std::get<1>(pt), 45.0));
}

MINI_TEST("Vector", "projection") {
    // uncomment #include "vector.h"

    // Project vector v=(1,1,1) onto each axis
    // Returns: (projection_vector, scalar_length, perpendicular_vector, perp_length)
    Vector v(1.0, 1.0, 1.0);
    Vector x = Vector::x_axis();
    Vector y = Vector::y_axis();
    Vector z = Vector::z_axis();
    auto [proj_x, lenx, perp_x, perp_lenx] = v.projection(x);
    auto [proj_y, leny, perp_y, perp_leny] = v.projection(y);
    auto [proj_z, lenz, perp_z, perp_lenz] = v.projection(z);

    MINI_CHECK(proj_x[0] == 1.0 && proj_x[1] == 0.0 && proj_x[2] == 0.0);
    MINI_CHECK(proj_y[0] == 0.0 && proj_y[1] == 1.0 && proj_y[2] == 0.0);
    MINI_CHECK(proj_z[0] == 0.0 && proj_z[1] == 0.0 && proj_z[2] == 1.0);
}

MINI_TEST("Vector", "is_parallel_to") {
    // uncomment #include "vector.h"

    // is_parallel_to returns: 1 (parallel), -1 (anti-parallel), 0 (not parallel)
    Vector v1(2.0, 2.0, 2.0);
    Vector v2(4.0, 4.0, 4.0);      // parallel (same direction)
    Vector v3(-1.0, -1.0, -1.0);   // anti-parallel (opposite direction)
    Vector v4(1.0, 0.0, 0.0);      // not parallel

    MINI_CHECK(v1.is_parallel_to(v2) == 1);
    MINI_CHECK(v1.is_parallel_to(v3) == -1);
    MINI_CHECK(v1.is_parallel_to(v4) == 0);
}

MINI_TEST("Vector", "is_perpendicular_to") {
    // uncomment #include "vector.h"

    // is_perpendicular_to: checks if two vectors are perpendicular (dot product ~ 0)
    Vector v1(1.0, 0.0, 0.0);
    Vector v2(0.0, 1.0, 0.0);  // perpendicular
    Vector v3(0.0, 0.0, 1.0);  // perpendicular
    Vector v4(1.0, 1.0, 0.0);  // not perpendicular

    // perpendicular_to: sets a vector to be perpendicular to another
    Vector z_axis(0.0, 0.0, 1.0);
    Vector x_axis = Vector::zero();
    // x_axis is now perpendicular to z_axis
    x_axis.perpendicular_to(z_axis);

    Vector arbitrary(1.0, 2.0, 3.0);
    Vector perp = Vector::zero();
    // perp is now perpendicular to arbitrary
    perp.perpendicular_to(arbitrary);

    MINI_CHECK(v1.is_perpendicular_to(v2));
    MINI_CHECK(v1.is_perpendicular_to(v3));
    MINI_CHECK(!v1.is_perpendicular_to(v4));
    MINI_CHECK(x_axis.is_perpendicular_to(z_axis));
    MINI_CHECK(perp.is_perpendicular_to(arbitrary));
}

MINI_TEST("Vector", "get_leveled_vector") {
    // uncomment #include "vector.h"

    // Scale vector along its direction so its Z-component equals vertical_height.
    Vector v(1.0, 1.0, 1.0);
    double vertical_height = 1.0;
    Vector v_leveled = v.get_leveled_vector(vertical_height);

    MINI_CHECK(TOLERANCE.is_close(v_leveled.magnitude(), std::sqrt(3.0)));
}

MINI_TEST("Vector", "cos_sin_laws") {
    // uncomment #include "vector.h"

    // Given a 3-4-5 right triangle
    double a = 3.0;  // side opposite to angle A
    double b = 4.0;  // side opposite to angle B
    double c = 5.0;  // hypotenuse, opposite to angle C (90 deg)

    // angle_from_cosine_law(adj1, adj2, opposite) -> angle opposite to 'opposite' side
    double angle_a = Vector::angle_from_cosine_law(b, c, a, true);
    double angle_b = Vector::angle_from_cosine_law(a, c, b, true);
    double angle_c = Vector::angle_from_cosine_law(a, b, c, true);

    // given 2 angles + 1 side, find other side
    double side_a = Vector::side_from_sine_law(angle_a, angle_b, b, true);
    double side_b = Vector::side_from_sine_law(angle_b, angle_c, c, true);
    double side_c = Vector::side_from_sine_law(angle_c, angle_a, a, true);

    // given 2 sides + included angle, find 3rd side
    double computed_c = Vector::cosine_law(a, b, angle_c, true);
    double computed_a = Vector::cosine_law(b, c, angle_a, true);
    double computed_b = Vector::cosine_law(a, c, angle_b, true);

    // given 2 sides + 1 angle, find other angle
    double computed_angle_b = Vector::sine_law_angle(a, angle_a, b, true);
    double computed_angle_a = Vector::sine_law_angle(b, angle_b, a, true);

    // given 1 side + 2 angles, find other side
    double computed_side_b = Vector::sine_law_length(a, angle_a, angle_b, true);
    double computed_side_a = Vector::sine_law_length(b, angle_b, angle_a, true);

    MINI_CHECK(TOLERANCE.is_close(angle_a, 36.86989764584402));
    MINI_CHECK(TOLERANCE.is_close(angle_b, 53.13010235415599));
    MINI_CHECK(TOLERANCE.is_close(angle_c, 90.0));
    MINI_CHECK(TOLERANCE.is_close(angle_a + angle_b + angle_c, 180.0));
    MINI_CHECK(TOLERANCE.is_close(side_a, a));
    MINI_CHECK(TOLERANCE.is_close(side_b, b));
    MINI_CHECK(TOLERANCE.is_close(side_c, c));
    MINI_CHECK(TOLERANCE.is_close(computed_c, c));
    MINI_CHECK(TOLERANCE.is_close(computed_a, a));
    MINI_CHECK(TOLERANCE.is_close(computed_b, b));
    MINI_CHECK(TOLERANCE.is_close(computed_angle_b, angle_b));
    MINI_CHECK(TOLERANCE.is_close(computed_angle_a, angle_a));
    MINI_CHECK(TOLERANCE.is_close(computed_side_b, b));
    MINI_CHECK(TOLERANCE.is_close(computed_side_a, a));
}

MINI_TEST("Vector", "sum_of_vectors") {
    // uncomment #include "vector.h"
    // uncomment #include <vector>

    // Sum of multiple vectors
    std::vector<Vector> vecs = {
        Vector(1.0, 1.0, 1.0),
        Vector(2.0, 2.0, 2.0),
        Vector(3.0, 3.0, 3.0),
    };
    Vector sum_v = Vector::sum_of_vectors(vecs);
    MINI_CHECK(sum_v[0] == 6.0);
    MINI_CHECK(sum_v[1] == 6.0);
    MINI_CHECK(sum_v[2] == 6.0);

    // Empty list returns zero vector
    std::vector<Vector> empty;
    Vector zero = Vector::sum_of_vectors(empty);
    MINI_CHECK(zero[0] == 0.0);
    MINI_CHECK(zero[1] == 0.0);
    MINI_CHECK(zero[2] == 0.0);
}

MINI_TEST("Vector", "average") {
    // uncomment #include "vector.h"
    // uncomment #include <vector>

    // Average of multiple vectors
    std::vector<Vector> vecs = {
        Vector(1.0, 2.0, 3.0),
        Vector(3.0, 4.0, 5.0),
        Vector(5.0, 6.0, 7.0),
    };
    Vector avg = Vector::average(vecs);
    MINI_CHECK(avg[0] == 3.0);
    MINI_CHECK(avg[1] == 4.0);
    MINI_CHECK(avg[2] == 5.0);
}

MINI_TEST("Vector", "is_zero") {
    // uncomment #include "vector.h"

    Vector zero(0.0, 0.0, 0.0);
    Vector nonzero(1.0, 0.0, 0.0);
    Vector tiny(1e-13, 1e-13, 1e-13);  // Magnitude ~ 1.7e-13 < 1e-12

    MINI_CHECK(zero.is_zero());
    MINI_CHECK(!nonzero.is_zero());
    MINI_CHECK(tiny.is_zero());
}

MINI_TEST("Vector", "json_roundtrip") {
    // uncomment #include "vector.h"

    Vector v(42.1, 84.2, 126.3);
    v.name = "test_vector";

    // json_dump(filename) / json_load(filename) - file-based serialization
    std::string filename = "test_vector.json";
    v.json_dump(filename);
    Vector loaded = Vector::json_load(filename);

    MINI_CHECK(loaded.name == "test_vector");
    MINI_CHECK(TOLERANCE.is_close(loaded[0], 42.1));
    MINI_CHECK(TOLERANCE.is_close(loaded[1], 84.2));
    MINI_CHECK(TOLERANCE.is_close(loaded[2], 126.3));
}

MINI_TEST("Vector", "protobuf_roundtrip") {
    // uncomment #include "vector.h"

    Vector v(42.1, 84.2, 126.3);
    v.name = "test_vector";

    // protobuf_dump(filename) / protobuf_load(filename) - file-based serialization
    std::string filename = "test_vector.bin";
    v.protobuf_dump(filename);
    Vector loaded = Vector::protobuf_load(filename);

    MINI_CHECK(loaded.name == "test_vector");
    MINI_CHECK(TOLERANCE.is_close(loaded[0], 42.1));
    MINI_CHECK(TOLERANCE.is_close(loaded[1], 84.2));
    MINI_CHECK(TOLERANCE.is_close(loaded[2], 126.3));
}

} // namespace session_cpp
