#include <filesystem>
#include "marching_squares.h"
#include "polyline.h"
#include "session.h"
#include "session_config.h"
#include "color.h"
#include "treenode.h"

using namespace session_cpp;

static std::shared_ptr<TreeNode> add_group(Session& session, const std::string& n, int r, int g, int b) {
    auto node = std::make_shared<TreeNode>(n);
    node->color = Color(r, g, b);
    session.add(node);
    return node;
}

int main() {
    SESSION_CONFIG.explode_mesh_faces = false;
    Session session("MarchingSquares");
    std::string fp = (std::filesystem::path(__FILE__).parent_path().parent_path()
                      / "session_data" / "MarchingSquares.pb").string();

    // 1. Circle: iso-contour of 1 - (x^2 + y^2) = 0 → unit circle
    {
        auto grp = add_group(session, "Circle (r=1)", 80, 160, 220);
        auto segs = MarchingSquares::extract_from_func(
            [](double x, double y) { return 1.0 - (x * x + y * y); },
            {-1.5, 1.5}, {-1.5, 1.5}, 60, 60, 0.0
        );
        for (auto& pl : segs) {
            pl.linecolor = Color(80, 160, 220);
            session.add(session.add_polyline(std::make_shared<Polyline>(pl)), grp);
        }
    }

    // 2. Saddle: iso-contour of x^2 - y^2 = 0 → crossing diagonals (hyperbolas)
    {
        auto grp = add_group(session, "Saddle (x^2-y^2=0)", 220, 120, 60);
        auto segs = MarchingSquares::extract_from_func(
            [](double x, double y) { return x * x - y * y; },
            {-2.0, 2.0}, {-2.0, 2.0}, 60, 60, 0.0
        );
        for (auto& pl : segs) {
            pl.linecolor = Color(220, 120, 60);
            session.add(session.add_polyline(std::make_shared<Polyline>(pl)), grp);
        }
    }

    // 3. Sine wave: iso-contour of sin(x)*cos(y) = 0 → wave grid
    {
        auto grp = add_group(session, "Sin*Cos wave", 100, 200, 100);
        auto segs = MarchingSquares::extract_from_func(
            [](double x, double y) { return std::sin(x) * std::cos(y); },
            {-6.3, 6.3}, {-6.3, 6.3}, 100, 100, 0.0
        );
        for (auto& pl : segs) {
            pl.linecolor = Color(100, 200, 100);
            session.add(session.add_polyline(std::make_shared<Polyline>(pl)), grp);
        }
    }

    // 4. Grid example: extract from scalar grid (Gaussian bump)
    {
        auto grp = add_group(session, "Gaussian bump (grid)", 200, 80, 200);
        int n = 40;
        std::vector<std::vector<double>> grid(n, std::vector<double>(n));
        for (int r = 0; r < n; ++r) {
            for (int c = 0; c < n; ++c) {
                double x = (c - n / 2.0) * 0.2;
                double y = (r - n / 2.0) * 0.2;
                grid[r][c] = std::exp(-(x * x + y * y));
            }
        }
        for (double iso : {0.2, 0.4, 0.6, 0.8}) {
            auto segs = MarchingSquares::extract(grid, iso, 0.2);
            int ci = static_cast<int>(iso * 255.0);
            for (auto& pl : segs) {
                pl.linecolor = Color(ci, 80, 200 - ci);
                session.add(session.add_polyline(std::make_shared<Polyline>(pl)), grp);
            }
        }
    }

    session.pb_dump(fp);
    return 0;
}
