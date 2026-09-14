#include "mini_test.h"
#include "tolerance.h"
#include "point.h"
#include "tolerance.pb.h"
#include "vector.h"
#include <cstdint>
#include <limits>
#include <stdexcept>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Tolerance", "Is Zero") {
        bool result = TOLERANCE.is_zero(1e-10);

        MINI_CHECK(result);
    }

    MINI_TEST("Tolerance", "Is Close") {
        bool result = TOLERANCE.is_close(1.0, 1.0 + 1e-7);

        MINI_CHECK(result);
    }

    MINI_TEST("Tolerance", "Is Positive") {
        bool result = TOLERANCE.is_positive(1.0);

        MINI_CHECK(result);
    }

    MINI_TEST("Tolerance", "Is Negative") {
        bool result = TOLERANCE.is_negative(-1.0);

        MINI_CHECK(result);
    }

    MINI_TEST("Tolerance", "Is Between") {
        bool result = TOLERANCE.is_between(0.5, 0.0, 1.0);

        MINI_CHECK(result);
    }

    MINI_TEST("Tolerance", "Format Number") {
        std::string result = TOLERANCE.format_number(3.14159, 2);

        MINI_CHECK(result == "3.14");
    }

    MINI_TEST("Tolerance", "Key") {
        std::string result = TOLERANCE.key(1.0, 2.0, 3.0);

        MINI_CHECK(result == "1.000,2.000,3.000");
    }

    MINI_TEST("Tolerance", "To Radians") {
        double r0 = Tolerance::to_radians(180.0);
        double r1 = Tolerance::to_radians(90.0);
        double r2 = Tolerance::to_radians(0.0);

        MINI_CHECK(std::abs(r0 - Tolerance::PI) < 1e-9);
        MINI_CHECK(std::abs(r1 - Tolerance::PI / 2.0) < 1e-9);
        MINI_CHECK(std::abs(r2) < 1e-9);
    }

    MINI_TEST("Tolerance", "To Degrees") {
        double d0 = Tolerance::to_degrees(Tolerance::PI);
        double d1 = Tolerance::to_degrees(Tolerance::PI / 2.0);
        double d2 = Tolerance::to_degrees(0.0);

        MINI_CHECK(std::abs(d0 - 180.0) < 1e-9);
        MINI_CHECK(std::abs(d1 - 90.0) < 1e-9);
        MINI_CHECK(std::abs(d2) < 1e-9);
    }

    MINI_TEST("Tolerance", "Runtime Modification") {
        Tolerance tolerance;
        const double original_absolute = tolerance.absolute();
        const double original_relative = tolerance.relative();

        MINI_CHECK(original_absolute == 1e-9);
        MINI_CHECK(original_relative == 1e-6);

        tolerance.set_absolute(1e-12);
        tolerance.set_relative(1e-12);
        MINI_CHECK(tolerance.absolute() == 1e-12);
        MINI_CHECK(tolerance.relative() == 1e-12);

        const bool close_with_tight = tolerance.is_close(1.0, 1.0 + 1e-11);
        MINI_CHECK(!close_with_tight);

        tolerance.reset();
        MINI_CHECK(tolerance.absolute() == 1e-9);
        MINI_CHECK(tolerance.relative() == 1e-6);

        const bool close_with_default = tolerance.is_close(1.0, 1.0 + 1e-11);
        MINI_CHECK(close_with_default);
    }

    MINI_TEST("Tolerance", "Json Roundtrip") {
        Tolerance tolerance("MM");
        tolerance.set_absolute(1e-8);
        tolerance.set_angular(2e-6);
        tolerance.set_angulardeflection(0.2);
        tolerance.set_approximation(0.002);
        tolerance.set_lineardeflection(0.003);
        tolerance.set_precision(4);
        tolerance.set_relative(3e-6);

        const std::string filename = "serialization/test_tolerance.json";
        tolerance.file_json_dump(filename);
        const Tolerance loaded = Tolerance::file_json_load(filename);
        const Tolerance parsed = Tolerance::file_json_loads(tolerance.file_json_dumps());

        MINI_CHECK(loaded.unit() == "MM");
        MINI_CHECK(loaded.absolute() == 1e-8);
        MINI_CHECK(loaded.angular() == 2e-6);
        MINI_CHECK(loaded.angulardeflection() == 0.2);
        MINI_CHECK(loaded.approximation() == 0.002);
        MINI_CHECK(loaded.lineardeflection() == 0.003);
        MINI_CHECK(loaded.precision() == 4);
        MINI_CHECK(loaded.relative() == 3e-6);
        MINI_CHECK(parsed.relative() == 3e-6);
    }

    MINI_TEST("Tolerance", "Protobuf Roundtrip") {
        Tolerance tolerance("MM");
        tolerance.set_absolute(1e-8);
        tolerance.set_angular(2e-6);
        tolerance.set_angulardeflection(0.2);
        tolerance.set_approximation(0.002);
        tolerance.set_lineardeflection(0.003);
        tolerance.set_precision(4);
        tolerance.set_relative(3e-6);

        const std::string filename = "serialization/test_tolerance.bin";
        tolerance.pb_dump(filename);
        const Tolerance loaded = Tolerance::pb_load(filename);
        const Tolerance parsed = Tolerance::pb_loads(tolerance.pb_dumps());
        const Tolerance converted = Tolerance::from_proto(tolerance.to_proto());

        MINI_CHECK(loaded.unit() == "MM");
        MINI_CHECK(loaded.absolute() == 1e-8);
        MINI_CHECK(loaded.angular() == 2e-6);
        MINI_CHECK(loaded.angulardeflection() == 0.2);
        MINI_CHECK(loaded.approximation() == 0.002);
        MINI_CHECK(loaded.lineardeflection() == 0.003);
        MINI_CHECK(loaded.precision() == 4);
        MINI_CHECK(loaded.relative() == 3e-6);
        MINI_CHECK(parsed.relative() == 3e-6);
        MINI_CHECK(converted.relative() == 3e-6);
    }

    MINI_TEST("Tolerance", "Serialization Errors") {
        const Tolerance tolerance;
        bool malformed = false;
        bool json_write_failed = false;
        bool pb_write_failed = false;

        try {
            Tolerance::pb_loads("\xff");
        } catch (const std::runtime_error&) {
            malformed = true;
        }
        try {
            tolerance.file_json_dump("");
        } catch (const std::runtime_error&) {
            json_write_failed = true;
        }
        try {
            tolerance.pb_dump("");
        } catch (const std::runtime_error&) {
            pb_write_failed = true;
        }

        MINI_CHECK(malformed);
        MINI_CHECK(json_write_failed);
        MINI_CHECK(pb_write_failed);
    }

    MINI_TEST("Tolerance", "Unique From Two Int") {
        uint64_t r0 = unique_from_two_int(3, 7);
        uint64_t r1 = unique_from_two_int(7, 3);

        MINI_CHECK(r0 == r1);
        MINI_CHECK(r0 == ((uint64_t(7) << 32) | uint64_t(3)));
    }

    MINI_TEST("Tolerance", "Wrap Index") {
        int r0 = wrap_index(0, 4);
        int r1 = wrap_index(3, 4);
        int r2 = wrap_index(4, 4);
        int r3 = wrap_index(-1, 4);
        int r4 = wrap_index(0, 0);

        MINI_CHECK(r0 == 0);
        MINI_CHECK(r1 == 3);
        MINI_CHECK(r2 == 0);
        MINI_CHECK(r3 == 3);
        MINI_CHECK(r4 == 0);
    }

    MINI_TEST("Tolerance", "Triangle Edge By Angle") {
        double r = triangle_edge_by_angle(1.0, 45.0);

        MINI_CHECK(std::abs(r - 1.0) < 1e-9);
        double r2 = triangle_edge_by_angle(5.0, 0.0);
        MINI_CHECK(std::abs(r2) < 1e-9);
    }

    MINI_TEST("Tolerance", "Rad Deg Conversion") {
        double r0 = rad_to_deg(Tolerance::PI);
        double r1 = deg_to_rad(180.0);
        double r2 = deg_to_rad(rad_to_deg(1.234));

        MINI_CHECK(std::abs(r0 - 180.0) < 1e-9);
        MINI_CHECK(std::abs(r1 - Tolerance::PI) < 1e-9);
        MINI_CHECK(std::abs(r2 - 1.234) < 1e-9);
    }

    MINI_TEST("Tolerance", "Count Digits") {
        int r0 = count_digits(0.0);
        int r1 = count_digits(1.0);
        int r2 = count_digits(9.9);
        int r3 = count_digits(10.0);
        int r4 = count_digits(100.5);
        int r5 = count_digits(-42.0);

        MINI_CHECK(r0 == 0);
        MINI_CHECK(r1 == 1);
        MINI_CHECK(r2 == 1);
        MINI_CHECK(r3 == 2);
        MINI_CHECK(r4 == 3);
        MINI_CHECK(r5 == 2);
    }

    MINI_TEST("Tolerance", "Is Angle Zero") {
        bool r0 = TOLERANCE.is_angle_zero(1e-8);
        bool r1 = TOLERANCE.is_angle_zero(0.1);

        MINI_CHECK(r0);
        MINI_CHECK(!r1);
    }

    MINI_TEST("Tolerance", "Is Angles Close") {
        bool r0 = TOLERANCE.is_angles_close(1.0, 1.0 + 1e-8);
        bool r1 = TOLERANCE.is_angles_close(1.0, 2.0);

        MINI_CHECK(r0);
        MINI_CHECK(!r1);
    }

    MINI_TEST("Tolerance", "Is Point Close") {
        Point a(1.0, 2.0, 3.0);
        Point b(1.0, 2.0, 3.0 + 1e-12);
        Point c(1.0, 2.0, 4.0);

        MINI_CHECK(TOLERANCE.is_point_close(a, b));
        MINI_CHECK(!TOLERANCE.is_point_close(a, c));
    }

    MINI_TEST("Tolerance", "Is Allclose") {
        std::vector<double> a = {1.0, 2.0, 3.0};
        std::vector<double> b = {1.0, 2.0, 3.0 + 1e-12};
        std::vector<double> c = {1.0, 2.0, 4.0};

        MINI_CHECK(TOLERANCE.is_allclose(a, b));
        MINI_CHECK(!TOLERANCE.is_allclose(a, c));
    }

    MINI_TEST("Tolerance", "Key Xy") {
        std::string result = TOLERANCE.key_xy(1.0, 2.0);

        MINI_CHECK(result == "1.000,2.000");
    }

    MINI_TEST("Tolerance", "Round To") {
        double r0 = Tolerance::round_to(3.14159, 2);
        double r1 = Tolerance::round_to(2.5, 0);

        MINI_CHECK(std::abs(r0 - 3.14) < 1e-9);
        MINI_CHECK(std::abs(r1 - 3.0) < 1e-9);
    }

    MINI_TEST("Tolerance", "Precision From Tolerance") {
        int prec = TOLERANCE.precision_from_tolerance();

        MINI_CHECK(prec == 9);
    }

    MINI_TEST("Tolerance", "Tolerance") {
        double result = TOLERANCE.tolerance(1.0, 1e-6, 1e-9);

        MINI_CHECK(std::abs(result - (1e-6 + 1e-9)) < 1e-18);
    }

    MINI_TEST("Tolerance", "Compare") {
        bool r0 = TOLERANCE.compare(1.0, 1.0 + 1e-7, 1e-6, 1e-9);
        bool r1 = TOLERANCE.compare(1.0, 2.0, 1e-6, 1e-9);

        MINI_CHECK(r0);
        MINI_CHECK(!r1);
    }

    MINI_TEST("Tolerance", "Is Finite") {
        bool r0 = is_finite(1.0);
        bool r1 = is_finite(std::numeric_limits<double>::infinity());

        MINI_CHECK(r0);
        MINI_CHECK(!r1);
    }

    MINI_TEST("Tolerance", "Is Vector Close") {
        Vector a(1.0, 2.0, 3.0);
        Vector b(1.0, 2.0, 3.0 + 1e-12);
        Vector c(1.0, 2.0, 4.0);

        MINI_CHECK(TOLERANCE.is_vector_close(a, b));
        MINI_CHECK(!TOLERANCE.is_vector_close(a, c));
    }

    MINI_TEST("Tolerance", "Temporary") {
        Tolerance tolerance;
        const double original = tolerance.absolute();
        bool inside = false;
        {
            auto guard = tolerance.temporary();
            guard->set_absolute(1e-12);
            inside = (*guard).absolute() == 1e-12;
        }
        const bool restored = tolerance.absolute() == original;

        bool threw = false;
        try {
            auto guard = tolerance.temporary();
            guard->set_absolute(1e-12);
            throw std::runtime_error("test");
        } catch (const std::runtime_error&) {
            threw = true;
        }
        const bool restored_after_error = tolerance.absolute() == original;

        MINI_CHECK(inside);
        MINI_CHECK(restored);
        MINI_CHECK(threw);
        MINI_CHECK(restored_after_error);
    }

}
