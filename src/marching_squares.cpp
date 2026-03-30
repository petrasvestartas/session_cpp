#include "marching_squares.h"
#include <cmath>

namespace session_cpp {

// For each case: list of (ea, eb) pairs, terminated by (-1,-1)
static const int EDGE_TABLE_RAW[16][5] = {
    {-1,-1, -1,-1, -1},  // 0000
    { 3, 0, -1,-1, -1},  // 0001
    { 0, 1, -1,-1, -1},  // 0010
    { 3, 1, -1,-1, -1},  // 0011
    { 1, 2, -1,-1, -1},  // 0100
    { 3, 2,  0, 1, -1},  // 0101 ambiguous
    { 0, 2, -1,-1, -1},  // 0110
    { 3, 2, -1,-1, -1},  // 0111
    { 2, 3, -1,-1, -1},  // 1000
    { 2, 0, -1,-1, -1},  // 1001
    { 2, 1,  3, 0, -1},  // 1010 ambiguous
    { 2, 1, -1,-1, -1},  // 1011
    { 1, 3, -1,-1, -1},  // 1100
    { 1, 0, -1,-1, -1},  // 1101
    { 0, 3, -1,-1, -1},  // 1110
    {-1,-1, -1,-1, -1},  // 1111
};

double MarchingSquares::interp(double a, double b, double va, double vb, double iso) {
    if (std::abs(vb - va) < 1e-12) return (a + b) * 0.5;
    double t = (iso - va) / (vb - va);
    return a + t * (b - a);
}

Point MarchingSquares::edge_pt(int e, double x0, double y0, double x1, double y1, double v0, double v1, double v2, double v3, double iso) {
    switch (e) {
        case 0: return Point(interp(x0, x1, v0, v1, iso), y0, 0.0);
        case 1: return Point(x1, interp(y0, y1, v1, v2, iso), 0.0);
        case 2: return Point(interp(x0, x1, v3, v2, iso), y1, 0.0);
        default: return Point(x0, interp(y0, y1, v0, v3, iso), 0.0);
    }
}

std::vector<Polyline> MarchingSquares::connect_segments(std::vector<std::pair<Point, Point>>& segs) {
    using Key = std::pair<long long, long long>;
    auto snap = [](double v) { return static_cast<long long>(std::round(v * 1e8)); };
    auto key  = [&](const Point& p) -> Key { return {snap(p[0]), snap(p[1])}; };

    std::map<Key, std::vector<std::pair<int, int>>> ep;
    for (int i = 0; i < (int)segs.size(); ++i) {
        ep[key(segs[i].first)].push_back({i, 0});
        ep[key(segs[i].second)].push_back({i, 1});
    }

    std::vector<bool> used(segs.size(), false);

    auto pop_next = [&](const Point& pt) -> std::optional<Point> {
        auto it = ep.find(key(pt));
        if (it == ep.end()) return std::nullopt;
        for (auto& [i, end] : it->second) {
            if (!used[i]) {
                used[i] = true;
                return end == 0 ? segs[i].second : segs[i].first;
            }
        }
        return std::nullopt;
    };

    std::vector<Polyline> result;
    for (int start = 0; start < (int)segs.size(); ++start) {
        if (used[start]) continue;
        used[start] = true;
        auto [a, b] = segs[start];
        std::vector<Point> forward = {b};
        while (auto nxt = pop_next(forward.back())) forward.push_back(*nxt);
        std::vector<Point> backward = {a};
        while (auto nxt = pop_next(backward.back())) backward.push_back(*nxt);
        std::vector<Point> pts;
        for (int i = (int)backward.size() - 1; i >= 0; --i) pts.push_back(backward[i]);
        for (auto& p : forward) pts.push_back(p);
        result.push_back(Polyline(pts));
    }
    return result;
}

std::vector<Polyline> MarchingSquares::extract(const std::vector<std::vector<double>>& grid, double iso_value, double cell_size) {
    int rows = static_cast<int>(grid.size());
    if (rows == 0) return {};
    int cols = static_cast<int>(grid[0].size());
    if (cols == 0) return {};
    std::vector<std::pair<Point, Point>> segs;
    for (int r = 0; r < rows - 1; ++r) {
        for (int c = 0; c < cols - 1; ++c) {
            double v0 = grid[r][c], v1 = grid[r][c+1], v2 = grid[r+1][c+1], v3 = grid[r+1][c];
            double x0 = c * cell_size, y0 = r * cell_size;
            double x1 = (c+1) * cell_size, y1 = (r+1) * cell_size;
            int ca = ((v3>=iso_value)?8:0)|((v2>=iso_value)?4:0)|((v1>=iso_value)?2:0)|((v0>=iso_value)?1:0);
            for (int i = 0; i < 5 && EDGE_TABLE_RAW[ca][i] != -1; i += 2) {
                int ea = EDGE_TABLE_RAW[ca][i], eb = EDGE_TABLE_RAW[ca][i+1];
                segs.push_back({edge_pt(ea, x0, y0, x1, y1, v0, v1, v2, v3, iso_value),
                                 edge_pt(eb, x0, y0, x1, y1, v0, v1, v2, v3, iso_value)});
            }
        }
    }
    return connect_segments(segs);
}

std::vector<Polyline> MarchingSquares::extract_from_func(
    const std::function<double(double, double)>& func,
    std::pair<double, double> x_range,
    std::pair<double, double> y_range,
    int nx, int ny,
    double iso_value
) {
    auto [x0, x1] = x_range;
    auto [y0, y1] = y_range;
    double dx = (nx > 1) ? (x1 - x0) / (nx - 1) : 0.0;
    double dy = (ny > 1) ? (y1 - y0) / (ny - 1) : 0.0;
    std::vector<std::vector<double>> grid(ny, std::vector<double>(nx));
    for (int r = 0; r < ny; ++r)
        for (int c = 0; c < nx; ++c)
            grid[r][c] = func(x0 + c * dx, y0 + r * dy);
    std::vector<std::pair<Point, Point>> segs;
    for (int r = 0; r < ny - 1; ++r) {
        for (int c = 0; c < nx - 1; ++c) {
            double v0 = grid[r][c], v1 = grid[r][c+1], v2 = grid[r+1][c+1], v3 = grid[r+1][c];
            double px0 = x0 + c * dx, py0 = y0 + r * dy;
            double px1 = x0 + (c+1) * dx, py1 = y0 + (r+1) * dy;
            int ca = ((v3>=iso_value)?8:0)|((v2>=iso_value)?4:0)|((v1>=iso_value)?2:0)|((v0>=iso_value)?1:0);
            for (int i = 0; i < 5 && EDGE_TABLE_RAW[ca][i] != -1; i += 2) {
                int ea = EDGE_TABLE_RAW[ca][i], eb = EDGE_TABLE_RAW[ca][i+1];
                segs.push_back({edge_pt(ea, px0, py0, px1, py1, v0, v1, v2, v3, iso_value),
                                 edge_pt(eb, px0, py0, px1, py1, v0, v1, v2, v3, iso_value)});
            }
        }
    }
    return connect_segments(segs);
}

} // namespace session_cpp
