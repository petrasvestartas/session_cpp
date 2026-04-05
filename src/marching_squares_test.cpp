#include "mini_test.h"
#include "marching_squares.h"
#include "tolerance.h"
#include <functional>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("MarchingSquares", "Extract") {
    // uncomment #include "marching_squares.h"
    std::vector<std::vector<double>> grid = {
        {0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0},
    };
    auto result = MarchingSquares::extract(grid, 0.5, 1.0);

    MINI_CHECK(result.size() == 1);
    MINI_CHECK(result[0].point_count() == 5);
    MINI_CHECK(result[0].is_closed());
}

MINI_TEST("MarchingSquares", "ExtractFromFunc") {
    // uncomment #include "marching_squares.h"
    auto result = MarchingSquares::extract_from_func(
        [](double x, double y) { return 1.0 - (x * x + y * y); },
        {-2.0, 2.0}, {-2.0, 2.0}, 20, 20, 0.0
    );

    MINI_CHECK(result.size() > 0);
}

MINI_TEST("MarchingSquares", "EmptyGrid") {
    // uncomment #include "marching_squares.h"
    std::vector<std::vector<double>> grid;
    auto result = MarchingSquares::extract(grid, 0.5, 1.0);

    MINI_CHECK(result.size() == 0);
}

MINI_TEST("MarchingSquares", "AllAbove") {
    // uncomment #include "marching_squares.h"
    std::vector<std::vector<double>> grid = {
        {2.0, 2.0, 2.0},
        {2.0, 2.0, 2.0},
        {2.0, 2.0, 2.0},
    };
    auto result = MarchingSquares::extract(grid, 1.0, 1.0);

    MINI_CHECK(result.size() == 0);
}

MINI_TEST("MarchingSquares", "AllBelow") {
    // uncomment #include "marching_squares.h"
    std::vector<std::vector<double>> grid = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
    };
    auto result = MarchingSquares::extract(grid, 1.0, 1.0);

    MINI_CHECK(result.size() == 0);
}

MINI_TEST("MarchingSquares", "Interpolation") {
    // uncomment #include "marching_squares.h"
    std::vector<std::vector<double>> grid = {
        {0.0, 2.0},
        {0.0, 2.0},
    };
    auto result = MarchingSquares::extract(grid, 1.0, 1.0);

    MINI_CHECK(result.size() == 1);
    auto seg = result[0];
    MINI_CHECK(TOLERANCE.is_close(seg.get_point(0)[0], 0.5));
    MINI_CHECK(TOLERANCE.is_close(seg.get_point(1)[0], 0.5));
}

} // namespace session_cpp
