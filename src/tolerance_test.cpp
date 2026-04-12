#include "mini_test.h"
#include "tolerance.h"
#include "point.h"
#include <cstdint>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Tolerance", "Is Zero") {
        // uncomment #include "tolerance.h"
        bool result = TOLERANCE.is_zero(1e-10);

        MINI_CHECK(result);
    }

    MINI_TEST("Tolerance", "Is Close") {
        // uncomment #include "tolerance.h"
        bool result = TOLERANCE.is_close(1.0, 1.0 + 1e-7);

        MINI_CHECK(result);
    }

    MINI_TEST("Tolerance", "Is Positive") {
        // uncomment #include "tolerance.h"
        bool result = TOLERANCE.is_positive(1.0);

        MINI_CHECK(result);
    }

    MINI_TEST("Tolerance", "Is Negative") {
        // uncomment #include "tolerance.h"
        bool result = TOLERANCE.is_negative(-1.0);

        MINI_CHECK(result);
    }

    MINI_TEST("Tolerance", "Is Between") {
        // uncomment #include "tolerance.h"
        bool result = TOLERANCE.is_between(0.5, 0.0, 1.0);

        MINI_CHECK(result);
    }

    MINI_TEST("Tolerance", "Format Number") {
        // uncomment #include "tolerance.h"
        std::string result = TOLERANCE.format_number(3.14159, 2);

        MINI_CHECK(result == "3.14");
    }

    MINI_TEST("Tolerance", "Key") {
        // uncomment #include "tolerance.h"
        std::string result = TOLERANCE.key(1.0, 2.0, 3.0);

        MINI_CHECK(result == "1.000,2.000,3.000");
    }

    MINI_TEST("Tolerance", "To Radians") {
        // uncomment #include "tolerance.h"
        MINI_CHECK(std::abs(Tolerance::to_radians(180.0) - Tolerance::PI) < 1e-9);
        MINI_CHECK(std::abs(Tolerance::to_radians(90.0) - Tolerance::PI / 2.0) < 1e-9);
        MINI_CHECK(std::abs(Tolerance::to_radians(0.0)) < 1e-9);
    }

    MINI_TEST("Tolerance", "To Degrees") {
        // uncomment #include "tolerance.h"
        MINI_CHECK(std::abs(Tolerance::to_degrees(Tolerance::PI) - 180.0) < 1e-9);
        MINI_CHECK(std::abs(Tolerance::to_degrees(Tolerance::PI / 2.0) - 90.0) < 1e-9);
        MINI_CHECK(std::abs(Tolerance::to_degrees(0.0)) < 1e-9);
    }

    MINI_TEST("Tolerance", "Runtime Modification") {
        // uncomment #include "tolerance.h"
        // Get current default values
        double original_absolute = TOLERANCE.absolute();
        double original_relative = TOLERANCE.relative();

        MINI_CHECK(original_absolute == 1e-9);
        MINI_CHECK(original_relative == 1e-6);

        // Modify tolerance values at runtime
        TOLERANCE.set_absolute(1e-12);
        TOLERANCE.set_relative(1e-12);
        MINI_CHECK(TOLERANCE.absolute() == 1e-12);
        MINI_CHECK(TOLERANCE.relative() == 1e-12);

        // Test with new tolerance - 1e-11 difference now fails is_close
        bool close_with_tight = TOLERANCE.is_close(1.0, 1.0 + 1e-11);
        MINI_CHECK(!close_with_tight);

        // Reset to defaults
        TOLERANCE.reset();
        MINI_CHECK(TOLERANCE.absolute() == 1e-9);
        MINI_CHECK(TOLERANCE.relative() == 1e-6);

        // Same test now passes with default tolerance
        bool close_with_default = TOLERANCE.is_close(1.0, 1.0 + 1e-11);
        MINI_CHECK(close_with_default);
    }

    MINI_TEST("Tolerance", "Unique From Two Int") {
        // uncomment #include "tolerance.h"
        uint64_t r0 = unique_from_two_int(3, 7);
        uint64_t r1 = unique_from_two_int(7, 3);

        MINI_CHECK(r0 == r1);
        MINI_CHECK(r0 == ((uint64_t(7) << 32) | uint64_t(3)));
    }

    MINI_TEST("Tolerance", "Wrap Index") {
        // uncomment #include "tolerance.h"
        MINI_CHECK(wrap_index(0, 4)  == 0);
        MINI_CHECK(wrap_index(3, 4)  == 3);
        MINI_CHECK(wrap_index(4, 4)  == 0);
        MINI_CHECK(wrap_index(-1, 4) == 3);
        MINI_CHECK(wrap_index(0, 0)  == 0);
    }

    MINI_TEST("Tolerance", "Triangle Edge By Angle") {
        // uncomment #include "tolerance.h"
        double r = triangle_edge_by_angle(1.0, 45.0);

        MINI_CHECK(std::abs(r - 1.0) < 1e-9);
        double r2 = triangle_edge_by_angle(5.0, 0.0);
        MINI_CHECK(std::abs(r2) < 1e-9);
    }

    MINI_TEST("Tolerance", "Rad Deg Conversion") {
        // uncomment #include "tolerance.h"
        MINI_CHECK(std::abs(rad_to_deg(Tolerance::PI) - 180.0) < 1e-9);
        MINI_CHECK(std::abs(deg_to_rad(180.0) - Tolerance::PI) < 1e-9);
        MINI_CHECK(std::abs(deg_to_rad(rad_to_deg(1.234)) - 1.234) < 1e-9);
    }

    MINI_TEST("Tolerance", "Count Digits") {
        // uncomment #include "tolerance.h"
        MINI_CHECK(count_digits(0.0)   == 0);
        MINI_CHECK(count_digits(1.0)   == 1);
        MINI_CHECK(count_digits(9.9)   == 1);
        MINI_CHECK(count_digits(10.0)  == 2);
        MINI_CHECK(count_digits(100.5) == 3);
        MINI_CHECK(count_digits(-42.0) == 2);
    }

    MINI_TEST("Tolerance", "Is Angle Zero") {
        // uncomment #include "tolerance.h"
        // Angular tolerance default is 1e-6
        MINI_CHECK(TOLERANCE.is_angle_zero(1e-8));
        MINI_CHECK(!TOLERANCE.is_angle_zero(0.1));
    }

    MINI_TEST("Tolerance", "Is Angles Close") {
        // uncomment #include "tolerance.h"
        MINI_CHECK(TOLERANCE.is_angles_close(1.0, 1.0 + 1e-8));
        MINI_CHECK(!TOLERANCE.is_angles_close(1.0, 2.0));
    }

    MINI_TEST("Tolerance", "Is Point Close") {
        // uncomment #include "tolerance.h"
        Point a(1.0, 2.0, 3.0);
        Point b(1.0, 2.0, 3.0 + 1e-12);
        Point c(1.0, 2.0, 4.0);

        MINI_CHECK(TOLERANCE.is_point_close(a, b));
        MINI_CHECK(!TOLERANCE.is_point_close(a, c));
    }

    MINI_TEST("Tolerance", "Is Allclose") {
        // uncomment #include "tolerance.h"
        std::vector<double> a = {1.0, 2.0, 3.0};
        std::vector<double> b = {1.0, 2.0, 3.0 + 1e-12};
        std::vector<double> c = {1.0, 2.0, 4.0};

        MINI_CHECK(TOLERANCE.is_allclose(a, b));
        MINI_CHECK(!TOLERANCE.is_allclose(a, c));
    }

    MINI_TEST("Tolerance", "Key Xy") {
        // uncomment #include "tolerance.h"
        std::string result = TOLERANCE.key_xy(1.0, 2.0);

        MINI_CHECK(result == "1.000,2.000");
    }

    MINI_TEST("Tolerance", "Round To") {
        // uncomment #include "tolerance.h"
        MINI_CHECK(std::abs(Tolerance::round_to(3.14159, 2) - 3.14) < 1e-9);
        MINI_CHECK(std::abs(Tolerance::round_to(2.5, 0) - 3.0) < 1e-9);
    }

    MINI_TEST("Tolerance", "Precision From Tolerance") {
        // uncomment #include "tolerance.h"
        // Default absolute tolerance is 1e-9 → precision should be 9
        int prec = TOLERANCE.precision_from_tolerance();

        MINI_CHECK(prec == 9);
    }

}
