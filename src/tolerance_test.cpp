#include "mini_test.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Tolerance", "is_zero") {
        bool result = TOLERANCE.is_zero(1e-10);
        MINI_CHECK(result == true);
    }

    MINI_TEST("Tolerance", "is_close") {
        bool result = TOLERANCE.is_close(1.0, 1.0 + 1e-7);
        MINI_CHECK(result == true);
    }

    MINI_TEST("Tolerance", "is_positive") {
        bool result = TOLERANCE.is_positive(1.0);
        MINI_CHECK(result == true);
    }

    MINI_TEST("Tolerance", "is_negative") {
        bool result = TOLERANCE.is_negative(-1.0);
        MINI_CHECK(result == true);
    }

    MINI_TEST("Tolerance", "is_between") {
        bool result = TOLERANCE.is_between(0.5, 0.0, 1.0);
        MINI_CHECK(result == true);
    }

    MINI_TEST("Tolerance", "format_number") {
        std::string result = TOLERANCE.format_number(3.14159, 2);
        MINI_CHECK(result == "3.14");
    }

    MINI_TEST("Tolerance", "key") {
        std::string result = TOLERANCE.key(1.0, 2.0, 3.0);
        MINI_CHECK(result == "1.000,2.000,3.000");
    }

}
