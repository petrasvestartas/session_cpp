#include "mini_test.h"
#include "tolerance.h"

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

}
