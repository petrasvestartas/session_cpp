#include "mini_test.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Tolerance", "Is_zero") {
        bool result = TOLERANCE.is_zero(1e-10);
        MINI_CHECK(result == true);
    }

    MINI_TEST("Tolerance", "Is_close") {
        bool result = TOLERANCE.is_close(1.0, 1.0 + 1e-7);
        MINI_CHECK(result == true);
    }

    MINI_TEST("Tolerance", "Is_positive") {
        bool result = TOLERANCE.is_positive(1.0);
        MINI_CHECK(result == true);
    }

    MINI_TEST("Tolerance", "Is_negative") {
        bool result = TOLERANCE.is_negative(-1.0);
        MINI_CHECK(result == true);
    }

    MINI_TEST("Tolerance", "Is_between") {
        bool result = TOLERANCE.is_between(0.5, 0.0, 1.0);
        MINI_CHECK(result == true);
    }

    MINI_TEST("Tolerance", "Format_number") {
        std::string result = TOLERANCE.format_number(3.14159, 2);
        MINI_CHECK(result == "3.14");
    }

    MINI_TEST("Tolerance", "Key") {
        std::string result = TOLERANCE.key(1.0, 2.0, 3.0);
        MINI_CHECK(result == "1.000,2.000,3.000");
    }

    MINI_TEST("Tolerance", "Runtime_modification") {
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
        MINI_CHECK(close_with_tight == false);

        // Reset to defaults
        TOLERANCE.reset();
        MINI_CHECK(TOLERANCE.absolute() == 1e-9);
        MINI_CHECK(TOLERANCE.relative() == 1e-6);

        // Same test now passes with default tolerance
        bool close_with_default = TOLERANCE.is_close(1.0, 1.0 + 1e-11);
        MINI_CHECK(close_with_default == true);
    }

}
