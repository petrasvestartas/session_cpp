#include <filesystem>
#include <fstream>
#include "session.h"
#include "element.h"
#include "polyline.h"
#include "rtree.h"
#include "aabb.h"
#include "json.h"
using namespace session_cpp;

static Vector polygon_normal(const std::vector<Point>& pts) {
    double nx = 0, ny = 0, nz = 0;
    size_t n = pts.size();
    for (size_t i = 0; i < n; ++i) {
        const auto& c = pts[i];
        const auto& np = pts[(i + 1) % n];
        nx += (c[1] - np[1]) * (c[2] + np[2]);
        ny += (c[2] - np[2]) * (c[0] + np[0]);
        nz += (c[0] - np[0]) * (c[1] + np[1]);
    }
    double mag = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (mag < 1e-12) return Vector(0, 0, 1);
    return Vector(nx / mag, ny / mag, nz / mag);
}

static std::vector<Point> open_points(const Polyline& pl) {
    auto pts = pl.get_points();
    if (pts.size() > 3) {
        auto& first = pts.front();
        auto& last = pts.back();
        if (std::abs(last[0] - first[0]) < 1e-6 &&
            std::abs(last[1] - first[1]) < 1e-6 &&
            std::abs(last[2] - first[2]) < 1e-6)
            pts.pop_back();
    }
    return pts;
}

int main() {
    auto base = std::filesystem::path(__FILE__).parent_path().parent_path();
    auto json_in = (base / "session_data" / "WoodStep1_data.json").string();
    auto pb_path = (base / "session_data" / "WoodStep2.pb").string();
    auto json_out = (base / "session_data" / "WoodStep2_data.json").string();

    // 1. Load polylines from Step 1
    std::ifstream fin(json_in);
    nlohmann::json data = nlohmann::json::parse(fin);
    std::vector<Polyline> polylines;
    for (auto& entry : data["polylines"]) {
        std::vector<Point> pts;
        for (auto& c : entry["coords"])
            pts.emplace_back(c[0].get<double>(), c[1].get<double>(), c[2].get<double>());
        polylines.emplace_back(pts);
    }
    fmt::print("Loaded {} polylines\n", polylines.size());

    // 2. Compute centroids and normals
    size_t N = polylines.size();
    std::vector<Point> centroids(N);
    std::vector<Vector> normals(N);
    for (size_t i = 0; i < N; i++) {
        auto pts = open_points(polylines[i]);
        double cx = 0, cy = 0, cz = 0;
        for (auto& p : pts) { cx += p[0]; cy += p[1]; cz += p[2]; }
        centroids[i] = Point(cx / pts.size(), cy / pts.size(), cz / pts.size());
        normals[i] = polygon_normal(pts);
    }

    // 3. Build RTree for pairing
    RTree<int, double, 3> tree;
    std::vector<AABB> aabbs(N);
    for (size_t i = 0; i < N; i++) {
        aabbs[i] = AABB::from_polyline(polylines[i], 500.0);
        double mn[3] = {aabbs[i].cx - aabbs[i].hx, aabbs[i].cy - aabbs[i].hy, aabbs[i].cz - aabbs[i].hz};
        double mx[3] = {aabbs[i].cx + aabbs[i].hx, aabbs[i].cy + aabbs[i].hy, aabbs[i].cz + aabbs[i].hz};
        tree.insert(mn, mx, static_cast<int>(i));
    }

    // 4. Pair polylines by spatial proximity + anti-parallel normals
    std::vector<bool> paired(N, false);
    std::vector<std::pair<int, int>> pairs;
    for (size_t i = 0; i < N; i++) {
        if (paired[i]) continue;
        int best_j = -1;
        double best_dist = std::numeric_limits<double>::max();

        double mn[3] = {aabbs[i].cx - aabbs[i].hx, aabbs[i].cy - aabbs[i].hy, aabbs[i].cz - aabbs[i].hz};
        double mx[3] = {aabbs[i].cx + aabbs[i].hx, aabbs[i].cy + aabbs[i].hy, aabbs[i].cz + aabbs[i].hz};

        auto callback = [&](int j) -> bool {
            if (j <= (int)i || paired[j]) return true;
            double dot = normals[i][0] * normals[j][0] + normals[i][1] * normals[j][1] + normals[i][2] * normals[j][2];
            if (std::abs(dot) < 0.7) return true;
            double dx = centroids[i][0] - centroids[j][0];
            double dy = centroids[i][1] - centroids[j][1];
            double dz = centroids[i][2] - centroids[j][2];
            double dist = dx * dx + dy * dy + dz * dz;
            if (dist < best_dist) {
                best_dist = dist;
                best_j = j;
            }
            return true;
        };
        tree.search(mn, mx, callback);

        if (best_j >= 0) {
            pairs.push_back({(int)i, best_j});
            paired[i] = paired[best_j] = true;
        }
    }
    fmt::print("Found {} pairs\n", pairs.size());

    // 5. Loft meshes from polyline pairs + store element data
    std::vector<Mesh> lofted_meshes;
    lofted_meshes.reserve(pairs.size());
    nlohmann::json elem_array = nlohmann::json::array();

    for (auto& [a, b] : pairs) {
        auto open_a = open_points(polylines[a]);
        auto open_b = open_points(polylines[b]);

        double thick = 0;
        size_t n = std::min(open_a.size(), open_b.size());
        for (size_t k = 0; k < n; k++) {
            double dx = open_a[k][0] - open_b[k][0];
            double dy = open_a[k][1] - open_b[k][1];
            double dz = open_a[k][2] - open_b[k][2];
            thick += std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        thick /= n;

        // Ensure open_a normal points toward open_b (consistent winding)
        size_t np = std::min(open_a.size(), open_b.size());
        Vector na = polygon_normal(open_a);
        double cax = 0, cay = 0, caz = 0, cbx = 0, cby = 0, cbz = 0;
        for (size_t k = 0; k < np; k++) {
            cax += open_a[k][0]; cay += open_a[k][1]; caz += open_a[k][2];
            cbx += open_b[k][0]; cby += open_b[k][1]; cbz += open_b[k][2];
        }
        double offset_dot = (cbx-cax)*na[0] + (cby-cay)*na[1] + (cbz-caz)*na[2];
        if (offset_dot < 0) std::swap(open_a, open_b);

        // Build mesh directly from bottom + top polyline points
        std::vector<Point> verts;
        verts.reserve(np * 2);
        for (size_t k = 0; k < np; k++) verts.push_back(open_a[k]);
        for (size_t k = 0; k < np; k++) verts.push_back(open_b[k]);
        std::vector<std::vector<size_t>> faces;
        std::vector<size_t> bottom_face;
        for (size_t k = np; k-- > 0;) bottom_face.push_back(k);
        faces.push_back(bottom_face);
        std::vector<size_t> top_face;
        for (size_t k = np; k < 2 * np; k++) top_face.push_back(k);
        faces.push_back(top_face);
        for (size_t k = 0; k < np; k++) {
            size_t k1 = (k + 1) % np;
            faces.push_back({k, k1, k1 + np, k + np});
        }
        Mesh plate = Mesh::from_vertices_and_faces(verts, faces);
        lofted_meshes.push_back(plate);

        // Store both polylines for downstream steps
        nlohmann::json e;
        nlohmann::json poly_bottom = nlohmann::json::array();
        for (auto& p : open_a) poly_bottom.push_back({p[0], p[1], p[2]});
        nlohmann::json poly_top = nlohmann::json::array();
        for (auto& p : open_b) poly_top.push_back({p[0], p[1], p[2]});
        e["polygon"] = poly_bottom;
        e["polygon_top"] = poly_top;
        e["thickness"] = thick;
        e["name"] = "plate_" + std::to_string(lofted_meshes.size() - 1);
        elem_array.push_back(e);
    }

    // 6. Visualize
    Session session("WoodStep2_Elements");
    auto g_mesh = session.add_group("Plate Meshes");
    for (auto& plate : lofted_meshes) {
        auto m = std::make_shared<Mesh>(plate);
        session.add(session.add_mesh(m), g_mesh);
    }
    session.pb_dump(pb_path);

    // 7. Write intermediate data
    nlohmann::ordered_json out_data;
    out_data["elements"] = elem_array;
    std::ofstream fout(json_out);
    fout << out_data.dump(2);

    fmt::print("Created {} plates, wrote {} and {}\n", lofted_meshes.size(), pb_path, json_out);
    return 0;
}
