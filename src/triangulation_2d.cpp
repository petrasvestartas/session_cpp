#include "triangulation_2d.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace session_cpp {

double Triangulation2D::cross_2d(double ax, double ay, double bx, double by, double cx, double cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

double Triangulation2D::signed_area_2d(const std::vector<double>& coords) {
    double area = 0.0;
    size_t n = coords.size() / 2;
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        area += coords[i * 2] * coords[j * 2 + 1];
        area -= coords[j * 2] * coords[i * 2 + 1];
    }
    return area * 0.5;
}

bool Triangulation2D::point_in_triangle_2d(double px, double py,
                                           double ax, double ay,
                                           double bx, double by,
                                           double cx, double cy) {
    double d1 = cross_2d(ax, ay, bx, by, px, py);
    double d2 = cross_2d(bx, by, cx, cy, px, py);
    double d3 = cross_2d(cx, cy, ax, ay, px, py);
    bool has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    bool has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    return !(has_neg && has_pos);
}

bool Triangulation2D::segments_intersect_proper(double ax, double ay, double bx, double by,
                                               double cx, double cy, double dx, double dy) {
    double d1 = cross_2d(cx, cy, dx, dy, ax, ay);
    double d2 = cross_2d(cx, cy, dx, dy, bx, by);
    double d3 = cross_2d(ax, ay, bx, by, cx, cy);
    double d4 = cross_2d(ax, ay, bx, by, dx, dy);
    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
        ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
        return true;
    return false;
}

bool Triangulation2D::is_visible(const std::vector<double>& coords, const std::vector<int>& polygon,
                                 int from_vertex, int to_vertex) {
    double ax = coords[from_vertex * 2], ay = coords[from_vertex * 2 + 1];
    double bx = coords[to_vertex * 2], by = coords[to_vertex * 2 + 1];
    for (size_t i = 0; i < polygon.size(); ++i) {
        size_t j = (i + 1) % polygon.size();
        int vi = polygon[i], vj = polygon[j];
        if (vi == from_vertex || vi == to_vertex || vj == from_vertex || vj == to_vertex) continue;
        double cx = coords[vi * 2], cy = coords[vi * 2 + 1];
        double dx = coords[vj * 2], dy = coords[vj * 2 + 1];
        if (segments_intersect_proper(ax, ay, bx, by, cx, cy, dx, dy))
            return false;
    }
    return true;
}

bool Triangulation2D::is_convex(const std::vector<double>& coords, const std::vector<int>& indices,
                                int prev, int curr, int next) {
    double ax = coords[indices[prev] * 2], ay = coords[indices[prev] * 2 + 1];
    double bx = coords[indices[curr] * 2], by = coords[indices[curr] * 2 + 1];
    double cx = coords[indices[next] * 2], cy = coords[indices[next] * 2 + 1];
    return cross_2d(ax, ay, bx, by, cx, cy) > 0.0;
}

bool Triangulation2D::is_ear(const std::vector<double>& coords, const std::vector<int>& indices,
                             const std::vector<bool>& reflex, int prev, int curr, int next) {
    double ax = coords[indices[prev] * 2], ay = coords[indices[prev] * 2 + 1];
    double bx = coords[indices[curr] * 2], by = coords[indices[curr] * 2 + 1];
    double cx = coords[indices[next] * 2], cy = coords[indices[next] * 2 + 1];

    for (size_t i = 0; i < indices.size(); ++i) {
        int ii = static_cast<int>(i);
        if (ii == prev || ii == curr || ii == next) continue;
        if (!reflex[i]) continue;
        double px = coords[indices[i] * 2], py = coords[indices[i] * 2 + 1];
        // Skip duplicate-coordinate vertices (bridge duplicates)
        if ((std::abs(px - ax) < 1e-12 && std::abs(py - ay) < 1e-12) ||
            (std::abs(px - bx) < 1e-12 && std::abs(py - by) < 1e-12) ||
            (std::abs(px - cx) < 1e-12 && std::abs(py - cy) < 1e-12))
            continue;
        if (point_in_triangle_2d(px, py, ax, ay, bx, by, cx, cy))
            return false;
    }
    return true;
}

std::vector<Triangle2D> Triangulation2D::ear_clip(const std::vector<double>& coords, std::vector<int> indices) {
    std::vector<Triangle2D> triangles;
    size_t n = indices.size();
    if (n < 3) return triangles;

    // Compute winding from the index polygon
    std::vector<double> poly_coords(n * 2);
    for (size_t i = 0; i < n; ++i) {
        poly_coords[i * 2] = coords[indices[i] * 2];
        poly_coords[i * 2 + 1] = coords[indices[i] * 2 + 1];
    }
    if (signed_area_2d(poly_coords) < 0.0) {
        std::reverse(indices.begin(), indices.end());
    }

    // Build reflex flags
    auto build_reflex = [&](const std::vector<int>& idx) {
        std::vector<bool> r(idx.size(), false);
        for (size_t i = 0; i < idx.size(); ++i) {
            int p = static_cast<int>((i + idx.size() - 1) % idx.size());
            int nx = static_cast<int>((i + 1) % idx.size());
            if (!is_convex(coords, idx, p, static_cast<int>(i), nx))
                r[i] = true;
        }
        return r;
    };

    std::vector<bool> reflex = build_reflex(indices);

    int max_iter = static_cast<int>(n * n * 2);
    int iter = 0;
    while (indices.size() > 3 && iter < max_iter) {
        bool found = false;
        for (size_t i = 0; i < indices.size(); ++i) {
            if (reflex[i]) continue;
            int p = static_cast<int>((i + indices.size() - 1) % indices.size());
            int nx = static_cast<int>((i + 1) % indices.size());
            if (is_ear(coords, indices, reflex, p, static_cast<int>(i), nx)) {
                triangles.push_back({indices[p], indices[i], indices[nx]});
                indices.erase(indices.begin() + static_cast<ptrdiff_t>(i));
                // Rebuild reflex flags
                reflex = build_reflex(indices);
                found = true;
                break;
            }
        }
        if (!found) {
            // Degenerate: remove a collinear vertex
            bool removed = false;
            for (size_t i = 0; i < indices.size(); ++i) {
                int p = static_cast<int>((i + indices.size() - 1) % indices.size());
                int nx = static_cast<int>((i + 1) % indices.size());
                double ax = coords[indices[p] * 2], ay = coords[indices[p] * 2 + 1];
                double bx = coords[indices[i] * 2], by = coords[indices[i] * 2 + 1];
                double cx = coords[indices[nx] * 2], cy = coords[indices[nx] * 2 + 1];
                double c = cross_2d(ax, ay, bx, by, cx, cy);
                if (std::abs(c) < 1e-12) {
                    triangles.push_back({indices[p], indices[static_cast<int>(i)], indices[nx]});
                    indices.erase(indices.begin() + static_cast<ptrdiff_t>(i));
                    reflex = build_reflex(indices);
                    removed = true;
                    break;
                }
            }
            if (!removed) break;
        }
        ++iter;
    }

    if (indices.size() == 3) {
        triangles.push_back({indices[0], indices[1], indices[2]});
    }

    return triangles;
}

std::vector<int> Triangulation2D::merge_holes(std::vector<double>& coords, std::vector<int>& boundary_indices,
                                              const std::vector<std::vector<int>>& hole_indices_list) {
    if (hole_indices_list.empty()) return boundary_indices;

    // Sort holes by maximum x of their vertices (descending)
    struct HoleInfo {
        int idx;
        double max_x;
        int max_x_local;
    };
    std::vector<HoleInfo> sorted_holes;
    for (size_t h = 0; h < hole_indices_list.size(); ++h) {
        const auto& hole = hole_indices_list[h];
        double mx = -std::numeric_limits<double>::max();
        int mx_local = 0;
        for (size_t i = 0; i < hole.size(); ++i) {
            double x = coords[hole[i] * 2];
            if (x > mx) { mx = x; mx_local = static_cast<int>(i); }
        }
        sorted_holes.push_back({static_cast<int>(h), mx, mx_local});
    }
    std::sort(sorted_holes.begin(), sorted_holes.end(),
              [](const HoleInfo& a, const HoleInfo& b) { return a.max_x > b.max_x; });

    std::vector<int> merged = boundary_indices;

    for (const auto& hi : sorted_holes) {
        const auto& hole = hole_indices_list[hi.idx];
        int hole_vi = hi.max_x_local;
        double hx = coords[hole[hole_vi] * 2];
        double hy = coords[hole[hole_vi] * 2 + 1];

        // Cast ray right from hole vertex, find closest boundary edge
        double best_ix = std::numeric_limits<double>::max();
        int best_edge = -1;
        for (size_t i = 0; i < merged.size(); ++i) {
            size_t j = (i + 1) % merged.size();
            double y0 = coords[merged[i] * 2 + 1];
            double y1 = coords[merged[j] * 2 + 1];
            double x0 = coords[merged[i] * 2];
            double x1 = coords[merged[j] * 2];

            if ((y0 <= hy && y1 > hy) || (y1 <= hy && y0 > hy)) {
                double t = (hy - y0) / (y1 - y0);
                double ix = x0 + t * (x1 - x0);
                if (ix >= hx && ix < best_ix) {
                    best_ix = ix;
                    best_edge = static_cast<int>(i);
                }
            }
        }

        if (best_edge < 0) continue;

        // Determine visible vertex
        size_t be0 = best_edge;
        size_t be1 = (best_edge + 1) % merged.size();
        int visible = -1;
        double vx0 = coords[merged[be0] * 2];
        double vx1 = coords[merged[be1] * 2];
        int candidate = (vx0 >= vx1) ? static_cast<int>(be0) : static_cast<int>(be1);

        // Check for vertices inside the triangle (hole_pt, intersection_pt, candidate_pt)
        // that are closer and visible
        double vis_x = coords[merged[candidate] * 2];
        double vis_y = coords[merged[candidate] * 2 + 1];
        double min_dist = std::numeric_limits<double>::max();
        visible = candidate;

        // Gather all polygon vertices inside the search triangle
        for (size_t i = 0; i < merged.size(); ++i) {
            double px = coords[merged[i] * 2];
            double py = coords[merged[i] * 2 + 1];
            if (px < hx) continue;
            if (point_in_triangle_2d(px, py, hx, hy, best_ix, hy, vis_x, vis_y) ||
                (std::abs(px - vis_x) < 1e-12 && std::abs(py - vis_y) < 1e-12)) {
                double dx = px - hx, dy = py - hy;
                double dist = dx * dx + dy * dy;
                if (dist < min_dist && is_visible(coords, merged, hole[hole_vi], merged[i])) {
                    min_dist = dist;
                    visible = static_cast<int>(i);
                }
            }
        }

        // Fallback: if no visible vertex found via triangle search, try all polygon vertices
        if (visible == candidate && !is_visible(coords, merged, hole[hole_vi], merged[candidate])) {
            min_dist = std::numeric_limits<double>::max();
            for (size_t i = 0; i < merged.size(); ++i) {
                double px = coords[merged[i] * 2];
                double py = coords[merged[i] * 2 + 1];
                double dx = px - hx, dy = py - hy;
                double dist = dx * dx + dy * dy;
                if (dist < min_dist && is_visible(coords, merged, hole[hole_vi], merged[i])) {
                    min_dist = dist;
                    visible = static_cast<int>(i);
                }
            }
        }

        // Splice hole into merged polygon at visible vertex
        std::vector<int> new_merged;
        for (int i = 0; i <= visible; ++i)
            new_merged.push_back(merged[i]);

        // Insert hole vertices starting from hole_vi, wrapping around
        int hole_n = static_cast<int>(hole.size());
        for (int i = 0; i <= hole_n; ++i)
            new_merged.push_back(hole[(hole_vi + i) % hole_n]);

        // Bridge back: duplicate the visible boundary vertex
        new_merged.push_back(merged[visible]);

        for (size_t i = visible + 1; i < merged.size(); ++i)
            new_merged.push_back(merged[i]);

        merged = new_merged;
    }

    return merged;
}

std::vector<Triangle2D> Triangulation2D::triangulate(const Polyline& boundary,
                                                     const std::vector<Polyline>& holes) {
    if (boundary.point_count() < 3) return {};

    // Extract 2D coords from boundary
    std::vector<double> coords;
    auto bpts = boundary.get_points();

    // Strip closing duplicate
    size_t bn = bpts.size();
    if (bn > 1 && std::abs(bpts[0][0] - bpts[bn - 1][0]) < 1e-12 &&
        std::abs(bpts[0][1] - bpts[bn - 1][1]) < 1e-12)
        --bn;

    std::vector<int> boundary_indices;
    for (size_t i = 0; i < bn; ++i) {
        int idx = static_cast<int>(coords.size() / 2);
        coords.push_back(bpts[i][0]);
        coords.push_back(bpts[i][1]);
        boundary_indices.push_back(idx);
    }

    // Ensure CCW boundary
    std::vector<double> bcoords(bn * 2);
    for (size_t i = 0; i < bn; ++i) {
        bcoords[i * 2] = coords[boundary_indices[i] * 2];
        bcoords[i * 2 + 1] = coords[boundary_indices[i] * 2 + 1];
    }
    if (signed_area_2d(bcoords) < 0.0)
        std::reverse(boundary_indices.begin(), boundary_indices.end());

    // Extract holes
    std::vector<std::vector<int>> hole_indices_list;
    for (const auto& hole : holes) {
        auto hpts = hole.get_points();
        size_t hn = hpts.size();
        if (hn < 3) continue;
        if (hn > 1 && std::abs(hpts[0][0] - hpts[hn - 1][0]) < 1e-12 &&
            std::abs(hpts[0][1] - hpts[hn - 1][1]) < 1e-12)
            --hn;

        std::vector<int> hole_indices;
        for (size_t i = 0; i < hn; ++i) {
            int idx = static_cast<int>(coords.size() / 2);
            coords.push_back(hpts[i][0]);
            coords.push_back(hpts[i][1]);
            hole_indices.push_back(idx);
        }

        // Ensure CW holes
        std::vector<double> hcoords(hn * 2);
        for (size_t i = 0; i < hn; ++i) {
            hcoords[i * 2] = coords[hole_indices[i] * 2];
            hcoords[i * 2 + 1] = coords[hole_indices[i] * 2 + 1];
        }
        if (signed_area_2d(hcoords) > 0.0)
            std::reverse(hole_indices.begin(), hole_indices.end());

        hole_indices_list.push_back(hole_indices);
    }

    // Merge holes into boundary
    std::vector<int> merged = merge_holes(coords, boundary_indices, hole_indices_list);

    // Ear clip
    return ear_clip(coords, merged);
}

} // namespace session_cpp
