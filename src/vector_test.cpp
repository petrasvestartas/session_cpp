#include "mini_test.h"
#include "vector.h"
#include "point.h"
#include "tolerance.h"
#include "xform.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Vector", "Constructor") {
        Vector v(1.0, 2.0, 3.0);
        Point p0(1.0, 2.0, 3.0);
        Point p1(2.0, 4.0, 6.0);
        Vector v_2p = Vector::from_points(p0, p1);

        v[0] = 10.0;
        v[1] = 20.0;
        v[2] = 30.0;

        double x = v[0];
        double y = v[1];
        double z = v[2];

        std::string vstr = v.str();
        std::string vrepr = v.repr();

        Vector vcopy = v;
        Vector vother(1.0, 2.0, 3.0);

        Vector vmult = v;
        vmult *= 2.0;
        Vector vdiv = v;
        vdiv /= 2.0;
        Vector vadd = v;
        vadd += Vector(1.0, 1.0, 1.0);
        Vector vsub = v;
        vsub -= Vector(1.0, 1.0, 1.0);

        Vector result_mul = v * 2.0;
        Vector result_div = v / 2.0;
        Vector result_add = v + Vector(1.0, 1.0, 1.0);
        Vector result_sub = v - Vector(1.0, 1.0, 1.0);
        Vector result_neg = -v;

        Vector vx = Vector::x_axis();
        Vector vy = Vector::y_axis();
        Vector vz = Vector::z_axis();
        Vector vzero = Vector::zero();

        MINI_CHECK(v.name == "my_vector");
        MINI_CHECK(v[0] == 10.0 && v[1] == 20.0 && v[2] == 30.0);
        MINI_CHECK(v.guid() != "");
        MINI_CHECK(x == 10.0 && y == 20.0 && z == 30.0);
        MINI_CHECK(v_2p[0] == 1.0 && v_2p[1] == 2.0 && v_2p[2] == 3.0);
        MINI_CHECK(vstr == "10.000000, 20.000000, 30.000000");
        MINI_CHECK(vrepr == "Vector(my_vector, 10.000000, 20.000000, 30.000000, 37.416574)");
        MINI_CHECK(vcopy == v && vcopy.guid() != v.guid());
        MINI_CHECK(vother != v);
        MINI_CHECK(vmult[0] == 20.0 && vmult[1] == 40.0 && vmult[2] == 60.0);
        MINI_CHECK(vdiv[0] == 5.0 && vdiv[1] == 10.0 && vdiv[2] == 15.0);
        MINI_CHECK(vadd[0] == 11.0 && vadd[1] == 21.0 && vadd[2] == 31.0);
        MINI_CHECK(vsub[0] == 9.0 && vsub[1] == 19.0 && vsub[2] == 29.0);
        MINI_CHECK(result_mul[0] == 20.0 && result_mul[1] == 40.0 && result_mul[2] == 60.0);
        MINI_CHECK(result_div[0] == 5.0 && result_div[1] == 10.0 && result_div[2] == 15.0);
        MINI_CHECK(result_add[0] == 11.0 && result_add[1] == 21.0 && result_add[2] == 31.0);
        MINI_CHECK(result_sub[0] == 9.0 && result_sub[1] == 19.0 && result_sub[2] == 29.0);
        MINI_CHECK(result_neg[0] == -10.0 && result_neg[1] == -20.0 && result_neg[2] == -30.0);
        MINI_CHECK(vx[0] == 1.0 && vx[1] == 0.0 && vx[2] == 0.0);
        MINI_CHECK(vy[0] == 0.0 && vy[1] == 1.0 && vy[2] == 0.0);
        MINI_CHECK(vz[0] == 0.0 && vz[1] == 0.0 && vz[2] == 1.0);
        MINI_CHECK(vzero[0] == 0.0 && vzero[1] == 0.0 && vzero[2] == 0.0);
    }

    MINI_TEST("Vector", "Transformation") {
        Vector v(1.0, 2.0, 3.0);
        Xform xform = Xform::translation(10.0, 20.0, 30.0);
        Vector moved = v.transformed(xform);
        v.transform(xform);

        Vector v2(1.0, 0.0, 0.0);
        Xform rotation = Xform::rotation_z(Tolerance::PI / 2.0);
        v2.transform(rotation);

        MINI_CHECK(moved[0] == 1.0 && moved[1] == 2.0 && moved[2] == 3.0);
        MINI_CHECK(v[0] == 1.0 && v[1] == 2.0 && v[2] == 3.0);
        MINI_CHECK(TOLERANCE.is_close(v2[0], 0.0) && TOLERANCE.is_close(v2[1], 1.0) && TOLERANCE.is_close(v2[2], 0.0));
    }

    MINI_TEST("Vector", "Magnitude") {
        Vector v(3.0, 4.0, 0.0);
        double length = v.magnitude();
        double length_squared = v.magnitude_squared();

        MINI_CHECK(length == 5.0);
        MINI_CHECK(length_squared == 25.0);
    }

    MINI_TEST("Vector", "Normalize") {
        Vector v0(3.0, 4.0, 0.0);
        bool ok = v0.normalize_self();

        Vector v1(3.0, 4.0, 0.0);
        Vector v2 = v1.normalized();

        Vector zero(0.0, 0.0, 0.0);
        bool zero_ok = zero.normalize_self();

        MINI_CHECK(ok && TOLERANCE.is_close(v0.magnitude(), 1.0));
        MINI_CHECK(TOLERANCE.is_close(v2.magnitude(), 1.0));
        MINI_CHECK(!zero_ok);
    }

    MINI_TEST("Vector", "Reverse") {
        Vector v(1.0, -2.0, 3.0);
        v.reverse();

        MINI_CHECK(v[0] == -1.0 && v[1] == 2.0 && v[2] == -3.0);
    }

    MINI_TEST("Vector", "Dot Product") {
        Vector v1(1.0, 0.0, 0.0);
        Vector v2(0.0, 1.0, 0.0);
        Vector v3(1.0, 0.0, 0.0);
        double dot_perp = v1.dot(v2);
        double dot_paral = v1.dot(v3);

        Vector a(3.0, 4.0, 0.0);
        Vector b(1.0, 0.0, 0.0);
        Vector b2(2.0, 0.0, 0.0);
        double proj_scalar = a.dot(b) / b.magnitude();
        double proj_coeff = a.dot(b2) / b2.magnitude_squared();

        MINI_CHECK(TOLERANCE.is_close(dot_perp, 0.0));
        MINI_CHECK(TOLERANCE.is_close(dot_paral, 1.0));
        MINI_CHECK(TOLERANCE.is_close(proj_scalar, 3.0));
        MINI_CHECK(TOLERANCE.is_close(proj_coeff, 1.5));
    }

    MINI_TEST("Vector", "Cross Product") {
        Vector v1(1.0, 0.0, 0.0);
        Vector v2(0.0, 1.0, 0.0);
        Vector vn = v1.cross(v2);

        Vector a(3.0, 0.0, 0.0);
        Vector b(0.0, 4.0, 0.0);
        double area = a.cross(b).magnitude();

        MINI_CHECK(vn[0] == 0.0 && vn[1] == 0.0 && vn[2] == 1.0);
        MINI_CHECK(TOLERANCE.is_close(area, 12.0));
    }

    MINI_TEST("Vector", "Angle") {
        Vector v1(1.0, 0.0, 0.0);
        Vector v2(0.0, 1.0, 0.0);
        Vector v3(1.0, 1.0, 0.0);
        double angle_90 = v1.angle(v2, false);
        double angle_45 = v1.angle(v3, false);
        double angle_signed = v2.angle(v1, true);
        double angle_rad = v1.angle(v2, false, false);

        Vector v_30(std::sqrt(3.0), 1.0, 0.0);
        Vector v_60(1.0, std::sqrt(3.0), 0.0);
        Vector v_135(-1.0, 1.0, 0.0);
        double xy_angle_30 = Vector::angle_between_vector_xy_components(v_30);
        double xy_angle_60 = Vector::angle_between_vector_xy_components(v_60);
        double xy_angle_135 = Vector::angle_between_vector_xy_components(v_135);

        Vector v_dir(35.4, 35.4, 86.6);
        std::array<double, 3> abg = v_dir.coordinate_direction_3angles(true);

        Vector v_sph(1.0, 1.0, std::sqrt(2.0));
        std::array<double, 2> pt = v_sph.coordinate_direction_2angles(true);

        MINI_CHECK(TOLERANCE.is_close(angle_90, 90.0));
        MINI_CHECK(TOLERANCE.is_close(angle_45, 45.0));
        MINI_CHECK(TOLERANCE.is_close(angle_signed, -90.0));
        MINI_CHECK(TOLERANCE.is_close(angle_rad, Tolerance::PI / 2.0));
        MINI_CHECK(TOLERANCE.is_close(xy_angle_30, 30.0));
        MINI_CHECK(TOLERANCE.is_close(xy_angle_60, 60.0));
        MINI_CHECK(TOLERANCE.is_close(xy_angle_135, 135.0));
        MINI_CHECK(TOLERANCE.is_close(abg[0], 69.2742));
        MINI_CHECK(TOLERANCE.is_close(abg[1], 69.2742));
        MINI_CHECK(TOLERANCE.is_close(abg[2], 30.032058));
        MINI_CHECK(TOLERANCE.is_close(pt[0], 45.0));
        MINI_CHECK(TOLERANCE.is_close(pt[1], 45.0));
    }

    MINI_TEST("Vector", "Projection") {
        Vector v(1.0, 1.0, 1.0);
        Vector x = Vector::x_axis();
        Vector y = Vector::y_axis();
        Vector z = Vector::z_axis();
        auto [proj_x, len_x, perp_x, perp_len_x] = v.projection(x);
        auto [proj_y, len_y, perp_y, perp_len_y] = v.projection(y);
        auto [proj_z, len_z, perp_z, perp_len_z] = v.projection(z);

        MINI_CHECK(proj_x[0] == 1.0 && proj_x[1] == 0.0 && proj_x[2] == 0.0);
        MINI_CHECK(proj_y[0] == 0.0 && proj_y[1] == 1.0 && proj_y[2] == 0.0);
        MINI_CHECK(proj_z[0] == 0.0 && proj_z[1] == 0.0 && proj_z[2] == 1.0);
        MINI_CHECK(TOLERANCE.is_close(len_x, 1.0) && TOLERANCE.is_close(len_y, 1.0) && TOLERANCE.is_close(len_z, 1.0));
        MINI_CHECK(perp_x[0] == 0.0 && perp_x[1] == 1.0 && perp_x[2] == 1.0);
        MINI_CHECK(TOLERANCE.is_close(perp_len_x, std::sqrt(2.0)));
    }

    MINI_TEST("Vector", "Is Parallel To") {
        Vector v1(2.0, 2.0, 2.0);
        Vector v2(4.0, 4.0, 4.0);
        Vector v3(-1.0, -1.0, -1.0);
        Vector v4(1.0, 0.0, 0.0);

        MINI_CHECK(v1.is_parallel_to(v2) == 1);
        MINI_CHECK(v1.is_parallel_to(v3) == -1);
        MINI_CHECK(v1.is_parallel_to(v4) == 0);
    }

    MINI_TEST("Vector", "Is Perpendicular To") {
        Vector v1(1.0, 0.0, 0.0);
        Vector v2(0.0, 1.0, 0.0);
        Vector v3(0.0, 0.0, 1.0);
        Vector v4(1.0, 1.0, 0.0);

        Vector z_axis(0.0, 0.0, 1.0);
        Vector x_axis = Vector::zero();
        bool x_ok = x_axis.perpendicular_to(z_axis);

        Vector arbitrary(1.0, 2.0, 3.0);
        Vector perp = Vector::zero();
        bool perp_ok = perp.perpendicular_to(arbitrary);

        MINI_CHECK(v1.is_perpendicular_to(v2));
        MINI_CHECK(v1.is_perpendicular_to(v3));
        MINI_CHECK(!v1.is_perpendicular_to(v4));
        MINI_CHECK(x_ok && x_axis.is_perpendicular_to(z_axis));
        MINI_CHECK(perp_ok && perp.is_perpendicular_to(arbitrary));
    }

    MINI_TEST("Vector", "Get Leveled Vector") {
        Vector v(1.0, 1.0, 1.0);
        double vertical_height = 1.0;
        Vector leveled = v.get_leveled_vector(vertical_height);

        MINI_CHECK(TOLERANCE.is_close(leveled.magnitude(), std::sqrt(3.0)));
        MINI_CHECK(TOLERANCE.is_close(leveled[2], vertical_height));
    }

    MINI_TEST("Vector", "Cos Sin Laws") {
        double a = 3.0;
        double b = 4.0;
        double c = 5.0;

        double angle_a = Vector::angle_from_cosine_law(b, c, a, true);
        double angle_b = Vector::angle_from_cosine_law(a, c, b, true);
        double angle_c = Vector::angle_from_cosine_law(a, b, c, true);

        double side_a = Vector::side_from_sine_law(angle_a, angle_b, b, true);
        double side_b = Vector::side_from_sine_law(angle_b, angle_c, c, true);
        double side_c = Vector::side_from_sine_law(angle_c, angle_a, a, true);

        double computed_c = Vector::cosine_law(a, b, angle_c, true);
        double computed_a = Vector::cosine_law(b, c, angle_a, true);
        double computed_b = Vector::cosine_law(a, c, angle_b, true);

        double computed_angle_b = Vector::sine_law_angle(a, angle_a, b, true);
        double computed_angle_a = Vector::sine_law_angle(b, angle_b, a, true);

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

    MINI_TEST("Vector", "Sum Of Vectors") {
        std::vector<Vector> vecs = {Vector(1.0, 1.0, 1.0), Vector(2.0, 2.0, 2.0), Vector(3.0, 3.0, 3.0)};
        Vector sum = Vector::sum_of_vectors(vecs);

        std::vector<Vector> empty;
        Vector zero = Vector::sum_of_vectors(empty);

        MINI_CHECK(sum[0] == 6.0 && sum[1] == 6.0 && sum[2] == 6.0);
        MINI_CHECK(zero[0] == 0.0 && zero[1] == 0.0 && zero[2] == 0.0);
    }

    MINI_TEST("Vector", "Average") {
        std::vector<Vector> vecs = {Vector(1.0, 2.0, 3.0), Vector(3.0, 4.0, 5.0), Vector(5.0, 6.0, 7.0)};
        Vector avg = Vector::average(vecs);

        std::vector<Vector> empty;
        Vector zero = Vector::average(empty);

        MINI_CHECK(avg[0] == 3.0 && avg[1] == 4.0 && avg[2] == 5.0);
        MINI_CHECK(zero[0] == 0.0 && zero[1] == 0.0 && zero[2] == 0.0);
    }

    MINI_TEST("Vector", "Is Zero") {
        Vector zero(0.0, 0.0, 0.0);
        Vector nonzero(1.0, 0.0, 0.0);
        Vector tiny(1e-13, 1e-13, 1e-13);

        MINI_CHECK(zero.is_zero());
        MINI_CHECK(!nonzero.is_zero());
        MINI_CHECK(tiny.is_zero());
    }

    MINI_TEST("Vector", "Scale") {
        Vector v(2.0, 4.0, 6.0);
        v.scale(0.5);
        Vector v_up(1.0, 2.0, 3.0);
        v_up.scale_up();
        Vector v_rt(1.0, 2.0, 3.0);
        v_rt.scale_up();
        v_rt.scale_down();

        MINI_CHECK(v[0] == 1.0 && v[1] == 2.0 && v[2] == 3.0);
        MINI_CHECK(v_up[0] == SCALE);
        MINI_CHECK(TOLERANCE.is_close(v_rt[0], 1.0) && TOLERANCE.is_close(v_rt[1], 2.0) && TOLERANCE.is_close(v_rt[2], 3.0));
    }

    MINI_TEST("Vector", "Reflect") {
        Vector v(1.0, 2.0, 3.0);
        Vector n = Vector::x_axis();
        Vector r = v.reflect(n);

        MINI_CHECK(TOLERANCE.is_close(r[0], -1.0));
        MINI_CHECK(TOLERANCE.is_close(r[1], 2.0));
        MINI_CHECK(TOLERANCE.is_close(r[2], 3.0));
    }

    MINI_TEST("Vector", "Average Normal") {
        std::vector<Point> square = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
            Point(0.0, 0.0, 0.0),
        };
        Vector n = Vector::average_normal(square);
        Vector empty = Vector::average_normal({});

        MINI_CHECK(TOLERANCE.is_close(std::abs(n[2]), 1.0));
        MINI_CHECK(TOLERANCE.is_close(n[0], 0.0) && TOLERANCE.is_close(n[1], 0.0));
        MINI_CHECK(empty.is_zero());
    }

    MINI_TEST("Vector", "Json Roundtrip") {
        Vector v(42.1, 84.2, 126.3);
        v.name = "test_vector";

        std::string filename = "serialization/test_vector.json";
        v.file_json_dump(filename);
        Vector loaded = Vector::file_json_load(filename);

        std::string json_string = v.file_json_dumps();
        Vector parsed = Vector::file_json_loads(json_string);

        MINI_CHECK(loaded.name == "test_vector");
        MINI_CHECK(loaded.guid() == v.guid());
        MINI_CHECK(TOLERANCE.is_close(loaded[0], 42.1));
        MINI_CHECK(TOLERANCE.is_close(loaded[1], 84.2));
        MINI_CHECK(TOLERANCE.is_close(loaded[2], 126.3));
        MINI_CHECK(parsed == v);
    }

    MINI_TEST("Vector", "Protobuf Roundtrip") {
        Vector v(42.1, 84.2, 126.3);
        v.name = "test_vector";

        std::string filename = "serialization/test_vector.bin";
        v.pb_dump(filename);
        Vector loaded = Vector::pb_load(filename);

        std::string data = v.pb_dumps();
        Vector parsed = Vector::pb_loads(data);

        MINI_CHECK(loaded.name == "test_vector");
        MINI_CHECK(TOLERANCE.is_close(loaded[0], 42.1));
        MINI_CHECK(TOLERANCE.is_close(loaded[1], 84.2));
        MINI_CHECK(TOLERANCE.is_close(loaded[2], 126.3));
        MINI_CHECK(parsed == v);
    }

} // namespace session_cpp
