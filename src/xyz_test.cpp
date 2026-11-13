#include "mini_test.h"
#include "xyz.h"
#include "point.h"
#include "vector.h"
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <type_traits>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Xyz", "Constructor") {
        // using session_cpp::xyz;

        xyz p{1.0, 2.0, 3.0};
        const xyz zero{};

        p[0] = 10.0;
        p[1] = 20.0;
        p[2] = 30.0;

        const double x = p[0];
        const double y = p[1];
        const double z = p[2];

        const xyz pcopy = p;
        const xyz pother{1.0, 2.0, 3.0};

        xyz padd = p;
        padd += xyz{1.0, 1.0, 1.0};

        xyz psub = p;
        psub -= xyz{1.0, 1.0, 1.0};

        xyz pmult = p;
        pmult *= 2.0;

        xyz pdiv = p;
        pdiv /= 2.0;

        const xyz result_add = p + pother;
        const xyz result_sub = p - pother;
        const xyz result_neg = -p;
        const xyz result_mul = p * 2.0;
        const xyz result_mul_left = 2.0 * p;
        const xyz result_div = p / 2.0;

        MINI_CHECK(sizeof(xyz) == 24 && alignof(xyz) == alignof(double));
        MINI_CHECK(std::is_trivially_copyable_v<xyz> && std::is_standard_layout_v<xyz> && std::is_aggregate_v<xyz>);
        MINI_CHECK(zero.x == 0.0 && zero.y == 0.0 && zero.z == 0.0);
        MINI_CHECK(p.x == 10.0 && p.y == 20.0 && p.z == 30.0);
        MINI_CHECK(x == 10.0 && y == 20.0 && z == 30.0);
        MINI_CHECK(&p[0] == &p.x && &p[1] == &p.y && &p[2] == &p.z);
        MINI_CHECK(pcopy == p);
        MINI_CHECK(pother != p);
        MINI_CHECK(padd == (xyz{11.0, 21.0, 31.0}));
        MINI_CHECK(psub == (xyz{9.0, 19.0, 29.0}));
        MINI_CHECK(pmult == (xyz{20.0, 40.0, 60.0}));
        MINI_CHECK(pdiv == (xyz{5.0, 10.0, 15.0}));
        MINI_CHECK(result_add == (xyz{11.0, 22.0, 33.0}));
        MINI_CHECK(result_sub == (xyz{9.0, 18.0, 27.0}));
        MINI_CHECK(result_neg == (xyz{-10.0, -20.0, -30.0}));
        MINI_CHECK(result_mul == (xyz{20.0, 40.0, 60.0}));
        MINI_CHECK(result_mul_left == result_mul);
        MINI_CHECK(result_div == (xyz{5.0, 10.0, 15.0}));
    }

    MINI_TEST("Xyz", "Dot") {
        // using session_cpp::xyz;

        const xyz a{1.0, 2.0, 3.0};
        const xyz b{4.0, -5.0, 6.0};

        MINI_CHECK(a.dot(b) == 12.0);
        MINI_CHECK(a.dot(b) == Vector(1.0, 2.0, 3.0).dot(Vector(4.0, -5.0, 6.0)));
    }

    MINI_TEST("Xyz", "Cross") {
        // using session_cpp::xyz;

        const xyz a{1.0, 2.0, 3.0};
        const xyz b{4.0, -5.0, 6.0};
        const xyz c = a.cross(b);
        const Vector v = Vector(1.0, 2.0, 3.0).cross(Vector(4.0, -5.0, 6.0));

        MINI_CHECK(c == (xyz{27.0, 6.0, -13.0}));
        MINI_CHECK(c.x == v[0] && c.y == v[1] && c.z == v[2]);
        MINI_CHECK(c.dot(a) == 0.0 && c.dot(b) == 0.0);
    }

    MINI_TEST("Xyz", "Length") {
        // using session_cpp::xyz;

        const xyz a{3.0, 4.0, 0.0};
        const xyz tiny{3e-300, 4e-300, 12e-300};
        const xyz huge{3e300, 4e300, 12e300};
        const xyz irrational{0.1, 0.2, 0.3};

        MINI_CHECK(a.length() == 5.0);
        MINI_CHECK(a.length_squared() == 25.0);
        MINI_CHECK(std::abs(tiny.length() / 1e-300 - 13.0) <= 1e-13);
        MINI_CHECK(std::abs(huge.length() / 1e300 - 13.0) <= 1e-13);
        MINI_CHECK((xyz{}).length() == 0.0);
        MINI_CHECK(irrational.length() == Vector(0.1, 0.2, 0.3).magnitude());
        MINI_CHECK(irrational.length_squared() == Vector(0.1, 0.2, 0.3).magnitude_squared());
    }

    MINI_TEST("Xyz", "Distance") {
        // using session_cpp::xyz;

        const xyz a{1.0, 2.0, 3.0};
        const xyz b{4.0, 6.0, 3.0};
        const xyz c{0.1, 0.2, 0.3};
        const xyz huge{3e300, 4e300, 0.0};

        MINI_CHECK(a.distance(b) == 5.0);
        MINI_CHECK(a.distance(a) == 0.0);
        MINI_CHECK(a.distance(c) == Point(1.0, 2.0, 3.0).distance(Point(0.1, 0.2, 0.3)));
        MINI_CHECK(std::abs((xyz{}).distance(huge) / 1e300 - 5.0) <= 1e-13);
    }

    MINI_TEST("Xyz", "Point And Vector") {
        // using session_cpp::xyz;
        // using session_cpp::Point;
        // using session_cpp::Vector;

        const Point point(1.5, -2.5, 3.5, "corner");
        const Vector vector(0.25, 0.5, -0.75);
        const xyz from_point = point;
        const xyz from_vector = vector;
        const Point to_point(from_point);
        const Vector to_vector(from_vector);
        const xyz difference = from_point - point;

        MINI_CHECK(from_point == (xyz{1.5, -2.5, 3.5}));
        MINI_CHECK(from_vector == (xyz{0.25, 0.5, -0.75}));
        MINI_CHECK(to_point[0] == 1.5 && to_point[1] == -2.5 && to_point[2] == 3.5);
        MINI_CHECK(to_point.name == "my_point" && to_point == Point(1.5, -2.5, 3.5));
        MINI_CHECK(to_vector[0] == 0.25 && to_vector[1] == 0.5 && to_vector[2] == -0.75);
        MINI_CHECK(to_vector == vector);
        MINI_CHECK(difference == (xyz{}));
        MINI_CHECK(sizeof(xyz) < sizeof(Vector) && sizeof(Vector) < sizeof(Point));
    }

    MINI_TEST("Xyz", "Xyzf") {
        // using session_cpp::xyzf;

        xyzf f{1.0f, 2.0f, 3.0f};
        f[2] = 4.5f;
        const xyzf converted = to_xyzf(xyz{0.1, 0.2, 0.3});

        MINI_CHECK(sizeof(xyzf) == 12 && std::is_trivially_copyable_v<xyzf> && std::is_standard_layout_v<xyzf>);
        MINI_CHECK(f[0] == 1.0f && f[1] == 2.0f && f[2] == 4.5f);
        MINI_CHECK(f == (xyzf{1.0f, 2.0f, 4.5f}));
        MINI_CHECK(f != (xyzf{1.0f, 2.0f, 3.0f}));
        MINI_CHECK(converted == (xyzf{0.1f, 0.2f, 0.3f}));
    }

    MINI_TEST("Xyz", "Ijk") {
        // using session_cpp::ijk;

        ijk t{0, 1, 2};
        t[2] = 7;

        MINI_CHECK(sizeof(ijk) == 12 && std::is_trivially_copyable_v<ijk> && std::is_standard_layout_v<ijk>);
        MINI_CHECK(t[0] == 0 && t[1] == 1 && t[2] == 7);
        MINI_CHECK(t == (ijk{0, 1, 7}));
        MINI_CHECK(t != (ijk{0, 1, 2}));
    }

    MINI_TEST("Xyz", "Flat") {
        // using session_cpp::xyz;
        // using session_cpp::flat;

        std::vector<xyz> points = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}};
        const std::vector<xyz>& fixed = points;
        const std::vector<xyzf> floats = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}};
        const std::vector<ijk> triangles = {{0, 1, 2}, {2, 3, 0}};
        const std::span<const double> values = flat(fixed);
        const std::span<const float> float_values = flat(floats);
        const std::span<const std::int32_t> indices = flat(triangles);
        bool in_order = true;
        for (std::size_t i = 0; i < values.size(); ++i)
            in_order = in_order && values[i] == static_cast<double>(i + 1);
        flat(points)[4] = 50.0;

        MINI_CHECK(values.size() == 9 && values.data() == &points[0].x);
        MINI_CHECK(in_order);
        MINI_CHECK(points[1].y == 50.0);
        MINI_CHECK(float_values.size() == 6 && float_values[3] == 4.0f && float_values.data() == &floats[0].x);
        MINI_CHECK(indices.size() == 6 && indices[4] == 3 && indices.data() == &triangles[0].i);
        MINI_CHECK(flat(std::vector<xyz>{}).empty());
    }

} // namespace session_cpp
