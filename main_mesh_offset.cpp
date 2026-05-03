#include <filesystem>
#include <algorithm>
#include <cmath>
#include "session.h"
#include "mesh.h"
#include "file_obj.h"
using namespace session_cpp;

// ── Helpers ───────────────────────────────────────────────────────────────────

static double det3(const double m[3][3]) {
    return m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
         - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
         + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
}

static bool solve3(const double A[3][3], const double b[3], double x[3]) {
    double d = det3(A);
    if (std::abs(d) < 1e-15) return false;
    for (int k = 0; k < 3; ++k) {
        double Ak[3][3];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                Ak[i][j] = (j == k) ? b[i] : A[i][j];
        x[k] = det3(Ak) / d;
    }
    return true;
}

// Dense Gauss-Jordan elimination on n×(n+1) augmented system (in-place).
static void gauss_jordan(std::vector<std::vector<double>>& A, int n) {
    for (int col = 0; col < n; ++col) {
        int piv = col;
        for (int r = col+1; r < n; ++r)
            if (std::abs(A[r][col]) > std::abs(A[piv][col])) piv = r;
        if (std::abs(A[piv][col]) < 1e-20) continue;
        std::swap(A[col], A[piv]);
        double inv = 1.0 / A[col][col];
        for (int j = col; j <= n; ++j) A[col][j] *= inv;
        for (int r = 0; r < n; ++r) {
            if (r == col) continue;
            double f = A[r][col];
            for (int j = col; j <= n; ++j) A[r][j] -= f * A[col][j];
        }
    }
}

// ── FEO Algorithm ─────────────────────────────────────────────────────────────
// Computes per-face offset distances d_i such that at every interior vertex,
// all adjacent offset planes n_i·x = h_i + d_i share a common point.
// This guarantees every offset face lies exactly on its offset plane (planar)
// and adjacent faces share edges directly (edge-to-edge, no wall quads).
static std::vector<double> feo_offsets(
    const Mesh& mesh,
    const std::vector<std::array<double,3>>& fn,
    const std::vector<double>& fh,
    const std::map<size_t,int>& fidx,
    const std::vector<size_t>& fkeys,
    double target_d)
{
    int F = (int)fkeys.size();

    // Constraint rows: each row is (sparse coefficients, rhs)
    struct Row { std::vector<std::pair<int,double>> c; double rhs; };
    std::vector<Row> rows;

    for (size_t vk : mesh.vertices()) {
        auto fks_opt = mesh.vertex_faces(vk);
        if (!fks_opt.has_value() || fks_opt->size() < 4) continue;
        const auto& vfks = *fks_opt;
        int n = (int)vfks.size();

        // Try to find 3 base faces with a non-singular normal matrix.
        // Use the first valid triple (i0, i1, i2).
        int b0 = -1, b1 = -1, b2 = -1;
        double N3T[3][3];
        for (int a = 0; a < n && b0 < 0; ++a) {
            for (int b = a+1; b < n && b0 < 0; ++b) {
                for (int c = b+1; c < n && b0 < 0; ++c) {
                    int ja = fidx.at(vfks[a]);
                    int jb = fidx.at(vfks[b]);
                    int jc = fidx.at(vfks[c]);
                    double M[3][3] = {
                        {fn[ja][0], fn[jb][0], fn[jc][0]},
                        {fn[ja][1], fn[jb][1], fn[jc][1]},
                        {fn[ja][2], fn[jb][2], fn[jc][2]},
                    };
                    if (std::abs(det3(M)) > 1e-6) {
                        b0 = a; b1 = b; b2 = c;
                        for (int i = 0; i < 3; ++i)
                            for (int j = 0; j < 3; ++j)
                                N3T[i][j] = M[i][j];
                    }
                }
            }
        }
        if (b0 < 0) continue; // degenerate vertex (all normals coplanar)

        int j0 = fidx.at(vfks[b0]);
        int j1 = fidx.at(vfks[b1]);
        int j2 = fidx.at(vfks[b2]);
        double h0 = fh[j0], h1 = fh[j1], h2 = fh[j2];

        // For every face not in the base triple: add FEO constraint.
        for (int k = 0; k < n; ++k) {
            if (k == b0 || k == b1 || k == b2) continue;
            int jk = fidx.at(vfks[k]);
            double nk[3] = {fn[jk][0], fn[jk][1], fn[jk][2]};
            double hk = fh[jk];
            // Solve N3^T w = n_k  (gives coefficients for the constraint row)
            double w[3];
            if (!solve3(N3T, nk, w)) continue;
            // Constraint: w[0]*d[j0] + w[1]*d[j1] + w[2]*d[j2] - d[jk] = hk - w·h
            Row row;
            row.c = {{j0, w[0]}, {j1, w[1]}, {j2, w[2]}, {jk, -1.0}};
            row.rhs = hk - w[0]*h0 - w[1]*h1 - w[2]*h2;
            rows.push_back(row);
        }
    }

    // Normalization: mean(d_i) = target_d
    {
        Row row;
        for (int i = 0; i < F; ++i) row.c.push_back({i, 1.0/F});
        row.rhs = target_d;
        rows.push_back(row);
    }

    // Soft regularization: pull each d_i toward target_d
    const double eps_reg = 0.01;
    for (int i = 0; i < F; ++i) {
        Row row;
        row.c = {{i, eps_reg}};
        row.rhs = eps_reg * target_d;
        rows.push_back(row);
    }

    // Build and solve normal equations: A^T A d = A^T b
    std::vector<std::vector<double>> AtA(F, std::vector<double>(F+1, 0.0));
    for (const auto& row : rows) {
        for (const auto& [ci, cvi] : row.c) {
            for (const auto& [cj, cvj] : row.c)
                AtA[ci][cj] += cvi * cvj;
            AtA[ci][F] += cvi * row.rhs;
        }
    }
    gauss_jordan(AtA, F);

    std::vector<double> d(F, target_d);
    for (int i = 0; i < F; ++i) d[i] = AtA[i][F];
    return d;
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main() {
    auto base = std::filesystem::path(__FILE__).parent_path().parent_path();
    const std::string pb_out = (base / "session_data" / "mesh_offset_panels.pb").string();
    const std::string obj_out = "C:/Users/Petras/Desktop/mesh_output.obj";

    auto polylines = file_obj::read_file_obj_polylines("C:/Users/Petras/Desktop/mesh_input.obj");
    std::vector<std::vector<Point>> polygons;
    for (const auto& pl : polylines) {
        auto pts = pl.get_points();
        if (pts.size() > 1) {
            double dx = pts.back()[0]-pts.front()[0];
            double dy = pts.back()[1]-pts.front()[1];
            double dz = pts.back()[2]-pts.front()[2];
            if (dx*dx+dy*dy+dz*dz < 1e-6) pts.pop_back();
        }
        if (pts.size() >= 3) polygons.push_back(pts);
    }
    Mesh mesh = Mesh::from_polylines(polygons);
    constexpr double target_d = 100.0;

    // Face index + normals + plane values
    auto fkeys = mesh.faces();
    int F = (int)fkeys.size();
    std::map<size_t,int> fidx;
    for (int i = 0; i < F; ++i) fidx[fkeys[i]] = i;

    std::vector<std::array<double,3>> fn(F);
    std::vector<double> fh(F);
    for (int i = 0; i < F; ++i) {
        auto n = mesh.face_normal(fkeys[i]).value();
        auto c = mesh.face_centroid(fkeys[i]).value();
        fn[i] = {n[0], n[1], n[2]};
        fh[i] = n[0]*c[0] + n[1]*c[1] + n[2]*c[2];
    }

    // Solve FEO system for per-face offsets
    auto d_off = feo_offsets(mesh, fn, fh, fidx, fkeys, target_d);

    // Compute offset vertex positions as exact 3-plane intersections.
    // At each vertex, use the first 3 adjacent offset planes that form a
    // non-singular system. By the FEO solution, the remaining planes pass
    // through the same point (within numerical precision).
    std::map<size_t, Point> off_verts;
    for (size_t vk : mesh.vertices()) {
        auto vp = mesh.vertex_point(vk);
        if (!vp.has_value()) continue;
        auto fks_opt = mesh.vertex_faces(vk);
        if (!fks_opt.has_value() || fks_opt->empty()) {
            off_verts[vk] = *vp;
            continue;
        }
        const auto& vfks = *fks_opt;
        int n = (int)vfks.size();

        // Build least-squares system using all adjacent planes (Tikhonov-regularized).
        // For valence ≤ 3 this is exact; for higher valence the FEO constraints
        // ensure the planes are (approximately) consistent.
        double mat[3][3] = {};
        double rhs[3] = {};
        const double eps = 1e-8;
        for (int k = 0; k < n; ++k) {
            int jk = fidx.at(vfks[k]);
            const auto& nk = fn[jk];
            double ck = fh[jk] + d_off[jk];
            for (int a = 0; a < 3; ++a) {
                for (int bj = 0; bj < 3; ++bj) mat[a][bj] += nk[a]*nk[bj];
                rhs[a] += nk[a]*ck;
            }
        }
        for (int a = 0; a < 3; ++a) {
            mat[a][a] += eps;
            rhs[a] += eps * (*vp)[a];
        }
        double px[3];
        if (solve3(mat, rhs, px)) off_verts[vk] = Point(px[0], px[1], px[2]);
        else                       off_verts[vk] = *vp;
    }

    // Build offset mesh (same topology, new vertex positions)
    Mesh result;
    std::map<size_t,size_t> vmap;
    for (size_t vk : mesh.vertices())
        vmap[vk] = result.add_vertex(off_verts.at(vk));
    for (size_t fk : fkeys) {
        auto fv = mesh.face_vertices(fk).value();
        std::vector<size_t> nvks;
        for (size_t vk : fv) nvks.push_back(vmap.at(vk));
        result.add_face(nvks);
    }

    Session session("MeshOffset");
    auto grp_orig = session.add_group("Original");
    auto grp_off  = session.add_group("Offset");
    {
        auto m = std::make_shared<Mesh>(mesh);
        m->set_objectcolor(Color(180, 180, 200));
        session.add_mesh(m, grp_orig);
    }
    {
        auto m = std::make_shared<Mesh>(result);
        m->set_objectcolor(Color(100, 140, 220));
        session.add_mesh(m, grp_off);
    }

    session.pb_dump(pb_out);
    file_obj::write_file_obj(result, obj_out);
    return 0;
}
