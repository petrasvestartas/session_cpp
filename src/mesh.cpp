#include "mesh.h"
#include "remesh_cdt.h"
#include "intersection.h"
#include "plane.h"
#include "fmt/core.h"
#include <fstream>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_map>
#include <bit>
#include <limits>
#include <thread>
#include <atomic>
#include "mesh.pb.h"
#include "color.pb.h"

namespace session_cpp {

std::vector<std::array<int,3>> cdt_triangulate(
    const std::vector<std::pair<double,double>>&,
    const std::vector<std::vector<std::pair<double,double>>>&);

// ═══════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════

static void parallel_for(size_t n, const std::function<void(size_t)>& fn) {
    const unsigned int hw = std::max(1u, std::thread::hardware_concurrency());
    const unsigned int nthreads = static_cast<unsigned int>(std::min((size_t)hw, n));
    std::atomic<size_t> idx{0};
    std::vector<std::thread> threads;
    threads.reserve(nthreads);
    for (unsigned int t = 0; t < nthreads; ++t)
        threads.emplace_back([&] {
            for (size_t i = idx.fetch_add(1); i < n; i = idx.fetch_add(1))
                fn(i);
        });
    for (std::thread& th : threads) th.join();
}

/// Unit Newell normal of a closed ring
static Vector newell_normal(const std::vector<Point>& pts) {
    const size_t n = pts.size();
    double nx = 0, ny = 0, nz = 0;
    for (size_t i = 0; i < n; ++i) {
        const Point& a = pts[i];
        const Point& b = pts[(i + 1) % n];
        nx += (a[1] - b[1]) * (a[2] + b[2]);
        ny += (a[2] - b[2]) * (a[0] + b[0]);
        nz += (a[0] - b[0]) * (a[1] + b[1]);
    }
    Vector normal(nx, ny, nz);
    if (!normal.normalize_self()) return Vector(0, 0, 0);
    return normal;
}

/// Average of a point ring
static Point ring_centroid(const std::vector<Point>& pts) {
    double x = 0, y = 0, z = 0;
    for (const Point& p : pts) { x += p[0]; y += p[1]; z += p[2]; }
    const double n = static_cast<double>(pts.size());
    return Point(x / n, y / n, z / n);
}

/// CDT of a planar ring projected onto its own plane; indices into pts, empty when degenerate
static std::vector<std::array<int,3>> planar_cdt(const std::vector<Point>& pts) {
    const size_t n = pts.size();
    double nx = 0, ny = 0, nz = 0;
    for (size_t i = 0; i < n; i++) {
        const Point& a = pts[i];
        const Point& b = pts[(i + 1) % n];
        nx += (a[1] - b[1]) * (a[2] + b[2]);
        ny += (a[2] - b[2]) * (a[0] + b[0]);
        nz += (a[0] - b[0]) * (a[1] + b[1]);
    }
    const double nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (nlen <= 1e-12) return {};
    nx /= nlen; ny /= nlen; nz /= nlen;
    double ux = 1, uy = 0, uz = 0;
    if (std::abs(nx) > 0.9) { ux = 0; uy = 1; }
    const double dot = ux * nx + uy * ny + uz * nz;
    ux -= dot * nx; uy -= dot * ny; uz -= dot * nz;
    const double um = std::sqrt(ux * ux + uy * uy + uz * uz);
    ux /= um; uy /= um; uz /= um;
    const double vx = ny * uz - nz * uy, vy = nz * ux - nx * uz, vz = nx * uy - ny * ux;
    std::vector<std::pair<double,double>> bpts;
    bpts.reserve(n);
    for (const Point& p : pts)
        bpts.push_back({p[0] * ux + p[1] * uy + p[2] * uz, p[0] * vx + p[1] * vy + p[2] * vz});
    return cdt_triangulate(bpts, {});
}

/// Twice the signed 2D area of a ring
static double signed_area_2d(const std::vector<std::pair<double,double>>& pts) {
    double area = 0.0;
    const size_t n = pts.size();
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        area += pts[i].first * pts[j].second - pts[j].first * pts[i].second;
    }
    return area;
}

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════

Mesh::Mesh() {
    default_vertex_attributes["x"] = 0.0;
    default_vertex_attributes["y"] = 0.0;
    default_vertex_attributes["z"] = 0.0;
}

Mesh::Mesh(const Mesh& other) {
    *this = other;
}

Mesh& Mesh::operator=(const Mesh& other) {
    if (this == &other) return *this;
    _guid = other._guid;
    name = other.name;
    halfedge = other.halfedge;
    vertex = other.vertex;
    face = other.face;
    face_holes = other.face_holes;
    facedata = other.facedata;
    edgedata = other.edgedata;
    default_vertex_attributes = other.default_vertex_attributes;
    default_face_attributes = other.default_face_attributes;
    default_edge_attributes = other.default_edge_attributes;
    pointcolors = other.pointcolors;
    facecolors = other.facecolors;
    linecolors = other.linecolors;
    widths = other.widths;
    objectcolor = other.objectcolor;
    color_mode = other.color_mode;
    max_vertex = other.max_vertex;
    max_face = other.max_face;
    triangulation = other.triangulation;
    clear_triangle_bvh();
    return *this;
}

bool Mesh::operator==(const Mesh& other) const {
    if (name != other.name) return false;
    if (vertex != other.vertex) return false;
    if (face != other.face) return false;
    return true;
}

bool Mesh::operator!=(const Mesh& other) const { return !(*this == other); }

Mesh::~Mesh() {}

Mesh Mesh::from_vertices_and_faces(const std::vector<Point>& vertices, const std::vector<std::vector<size_t>>& faces) {
    Mesh mesh;
    for (const Point& pt : vertices)
        mesh.add_vertex(pt);
    for (const std::vector<size_t>& f : faces)
        mesh.add_face(f);
    return mesh;
}

/// Vertex key of p, merged by precision grid when given and by exact bits otherwise
static size_t polylines_vertex_key(
    Mesh& mesh,
    const Point& p,
    std::optional<double> precision,
    std::map<std::tuple<int64_t, int64_t, int64_t>, size_t>& map_eps,
    std::map<std::tuple<uint64_t, uint64_t, uint64_t>, size_t>& map_exact
) {
    if (precision.has_value()) {
        const double eps = *precision;
        const std::tuple<int64_t, int64_t, int64_t> key(
            static_cast<int64_t>(std::round(p[0] / eps)),
            static_cast<int64_t>(std::round(p[1] / eps)),
            static_cast<int64_t>(std::round(p[2] / eps)));
        const auto it = map_eps.find(key);
        if (it != map_eps.end()) return it->second;
        const size_t vk = mesh.add_vertex(p);
        map_eps[key] = vk;
        return vk;
    }
    const std::tuple<uint64_t, uint64_t, uint64_t> key(
        std::bit_cast<uint64_t>(p[0]),
        std::bit_cast<uint64_t>(p[1]),
        std::bit_cast<uint64_t>(p[2]));
    const auto it = map_exact.find(key);
    if (it != map_exact.end()) return it->second;
    const size_t vk = mesh.add_vertex(p);
    map_exact[key] = vk;
    return vk;
}

Mesh Mesh::from_polylines(const std::vector<std::vector<Point>>& polygons, std::optional<double> precision) {
    Mesh mesh;
    std::map<std::tuple<int64_t, int64_t, int64_t>, size_t> map_eps;
    std::map<std::tuple<uint64_t, uint64_t, uint64_t>, size_t> map_exact;
    for (const std::vector<Point>& poly : polygons) {
        if (poly.size() < 3) continue;
        std::vector<size_t> vkeys;
        vkeys.reserve(poly.size());
        for (const Point& p : poly)
            vkeys.push_back(polylines_vertex_key(mesh, p, precision, map_eps, map_exact));
        if (vkeys.size() > 1 && vkeys.back() == vkeys.front())
            vkeys.pop_back();
        if (vkeys.size() < 3) continue;
        const std::optional<size_t> fk = mesh.add_face(vkeys);
        if (!fk || vkeys.size() < 4) continue;
        const std::vector<Point> ring(poly.begin(), poly.begin() + vkeys.size());
        const std::vector<std::array<int,3>> tris = planar_cdt(ring);
        if (tris.empty()) continue;
        std::vector<std::array<size_t,3>> tri_list;
        tri_list.reserve(tris.size());
        for (const std::array<int,3>& t : tris)
            tri_list.push_back({vkeys[t[0]], vkeys[t[1]], vkeys[t[2]]});
        mesh.triangulation[*fk] = tri_list;
    }
    return mesh;
}

Mesh Mesh::from_polylines(const std::vector<Polyline>& polylines, std::optional<double> precision) {
    std::vector<std::vector<Point>> polygons;
    polygons.reserve(polylines.size());
    for (const Polyline& polyline : polylines)
        polygons.push_back(polyline.get_points());
    return from_polylines(polygons, precision);
}

/// Grid spacing for merging line endpoints: the given precision or a millionth of the bbox diagonal
static double lines_precision(const std::vector<Point>& pts, std::optional<double> precision) {
    const double eps = precision.value_or(0.0);
    if (eps > 0.0) return eps;
    double minx = pts[0][0], miny = pts[0][1], minz = pts[0][2];
    double maxx = minx, maxy = miny, maxz = minz;
    for (const Point& p : pts) {
        minx = std::min(minx, p[0]); maxx = std::max(maxx, p[0]);
        miny = std::min(miny, p[1]); maxy = std::max(maxy, p[1]);
        minz = std::min(minz, p[2]); maxz = std::max(maxz, p[2]);
    }
    const double diag = std::sqrt((maxx-minx)*(maxx-minx) + (maxy-miny)*(maxy-miny) + (maxz-minz)*(maxz-minz));
    return std::max(diag * 1e-6, 1e-12);
}

/// Index of p in verts, appending it when its grid cell is new
static size_t lines_vertex_id(
    const Point& p,
    double eps,
    std::map<std::tuple<int64_t, int64_t, int64_t>, size_t>& vmap,
    std::vector<Point>& verts
) {
    const std::tuple<int64_t, int64_t, int64_t> key(
        static_cast<int64_t>(std::round(p[0] / eps)),
        static_cast<int64_t>(std::round(p[1] / eps)),
        static_cast<int64_t>(std::round(p[2] / eps)));
    const auto it = vmap.find(key);
    if (it != vmap.end()) return it->second;
    const size_t id = verts.size();
    verts.push_back(p);
    vmap[key] = id;
    return id;
}

/// Face cycles of a planar graph: from u->v the next edge turns to the CW predecessor of u around v
static std::vector<std::vector<size_t>> lines_face_cycles(std::map<size_t, std::vector<size_t>>& adj, size_t nv) {
    std::set<std::pair<size_t, size_t>> visited;
    std::vector<std::vector<size_t>> cycles;
    for (const auto& [u, nbrs] : adj) {
        for (size_t v : nbrs) {
            if (visited.count({u, v})) continue;
            std::vector<size_t> cycle;
            size_t cu = u, cv = v;
            bool valid = true;
            while (cycle.size() <= nv * 2) {
                if (visited.count({cu, cv})) break;
                visited.insert({cu, cv});
                cycle.push_back(cu);
                const std::vector<size_t>& cv_nbrs = adj[cv];
                const auto it = std::find(cv_nbrs.begin(), cv_nbrs.end(), cu);
                if (it == cv_nbrs.end()) { valid = false; break; }
                const size_t idx = static_cast<size_t>(it - cv_nbrs.begin());
                const size_t prev_idx = (idx == 0) ? cv_nbrs.size() - 1 : idx - 1;
                cu = cv;
                cv = cv_nbrs[prev_idx];
            }
            if (cycle.size() > nv * 2) valid = false;
            if (valid && cycle.size() >= 3) cycles.push_back(cycle);
        }
    }
    return cycles;
}

/// Index of the cycle with the most negative signed area: the outer boundary
static size_t lines_outer_cycle(const std::vector<std::vector<size_t>>& cycles, const std::vector<Point>& verts) {
    size_t min_idx = 0;
    double min_area = std::numeric_limits<double>::max();
    for (size_t i = 0; i < cycles.size(); ++i) {
        std::vector<std::pair<double,double>> pts;
        pts.reserve(cycles[i].size());
        for (size_t vid : cycles[i]) pts.push_back({verts[vid][0], verts[vid][1]});
        const double area = signed_area_2d(pts) * 0.5;
        if (area < min_area) { min_area = area; min_idx = i; }
    }
    return min_idx;
}

Mesh Mesh::from_lines(const std::vector<Line>& lines, bool delete_boundary_face, std::optional<double> precision) {
    if (lines.empty()) return Mesh();
    std::vector<Point> all_pts;
    all_pts.reserve(lines.size() * 2);
    for (const Line& ln : lines) {
        all_pts.push_back(ln.start());
        all_pts.push_back(ln.end());
    }
    const double eps = lines_precision(all_pts, precision);
    std::map<std::tuple<int64_t, int64_t, int64_t>, size_t> vmap;
    std::vector<Point> verts;
    std::map<size_t, std::vector<size_t>> adj;
    for (const Line& ln : lines) {
        const size_t a = lines_vertex_id(ln.start(), eps, vmap, verts);
        const size_t b = lines_vertex_id(ln.end(), eps, vmap, verts);
        if (a == b) continue;
        adj[a].push_back(b);
        adj[b].push_back(a);
    }
    for (auto& [v, nbrs] : adj) {
        std::sort(nbrs.begin(), nbrs.end());
        nbrs.erase(std::unique(nbrs.begin(), nbrs.end()), nbrs.end());
        const double vx = verts[v][0], vy = verts[v][1];
        std::sort(nbrs.begin(), nbrs.end(), [&](size_t a, size_t b) {
            return std::atan2(verts[a][1] - vy, verts[a][0] - vx) < std::atan2(verts[b][1] - vy, verts[b][0] - vx);
        });
    }
    std::vector<std::vector<size_t>> cycles = lines_face_cycles(adj, verts.size());
    if (delete_boundary_face && !cycles.empty())
        cycles.erase(cycles.begin() + lines_outer_cycle(cycles, verts));
    Mesh mesh;
    std::vector<size_t> vkeys;
    vkeys.reserve(verts.size());
    for (const Point& pt : verts) vkeys.push_back(mesh.add_vertex(pt));
    for (const std::vector<size_t>& cycle : cycles) {
        std::vector<size_t> fvkeys;
        fvkeys.reserve(cycle.size());
        for (size_t vid : cycle) fvkeys.push_back(vkeys[vid]);
        const std::optional<size_t> fk = mesh.add_face(fvkeys);
        if (!fk) continue;
        std::vector<size_t> ordered = cycle;
        std::vector<std::pair<double,double>> bpts;
        bpts.reserve(ordered.size());
        for (size_t vid : ordered) bpts.push_back({verts[vid][0], verts[vid][1]});
        if (signed_area_2d(bpts) < 0.0) {
            std::reverse(bpts.begin(), bpts.end());
            std::reverse(ordered.begin(), ordered.end());
        }
        const std::vector<std::array<int,3>> tris = cdt_triangulate(bpts, {});
        std::vector<std::array<size_t, 3>> tri_list;
        tri_list.reserve(tris.size());
        for (const std::array<int,3>& t : tris)
            tri_list.push_back({vkeys[ordered[t[0]]], vkeys[ordered[t[1]]], vkeys[ordered[t[2]]]});
        mesh.triangulation[*fk] = tri_list;
    }
    return mesh;
}

Mesh Mesh::from_polygon_with_holes(const std::vector<std::vector<Point>>& polylines, bool sort_by_bbox) {
    if (polylines.empty()) return Mesh();
    std::vector<Polyline> pls;
    pls.reserve(polylines.size());
    for (const std::vector<Point>& v : polylines) pls.emplace_back(v);
    return RemeshCDT::from_polylines(pls, false, !sort_by_bbox);
}

// ═══════════════════════════════════════════════════════════════════════════
// Loft
// ═══════════════════════════════════════════════════════════════════════════

struct LoftFrame { Point origin; Vector xaxis; Vector yaxis; };
struct LoftRing { size_t off; size_t n; };
struct LoftPoly { LoftRing bot; LoftRing top; };

static std::pair<double, double> loft_project(const LoftFrame& frame, const Point& p) {
    const double dx = p[0] - frame.origin[0];
    const double dy = p[1] - frame.origin[1];
    const double dz = p[2] - frame.origin[2];
    return {dx * frame.xaxis[0] + dy * frame.xaxis[1] + dz * frame.xaxis[2],
            dx * frame.yaxis[0] + dy * frame.yaxis[1] + dz * frame.yaxis[2]};
}

/// Polyline points without the closing duplicate
static std::vector<Point> loft_open_points(const Polyline& pl) {
    std::vector<Point> pts = pl.get_points();
    if (pts.size() > 1) {
        const Point& f = pts.front();
        const Point& b = pts.back();
        if (std::abs(f[0]-b[0]) < 1e-12 && std::abs(f[1]-b[1]) < 1e-12 && std::abs(f[2]-b[2]) < 1e-12)
            pts.pop_back();
    }
    return pts;
}

static double loft_signed_area(const LoftFrame& frame, const std::vector<Point>& pts) {
    std::vector<std::pair<double,double>> pts2d;
    pts2d.reserve(pts.size());
    for (const Point& p : pts) pts2d.push_back(loft_project(frame, p));
    return signed_area_2d(pts2d) * 0.5;
}

/// Index of the polyline with the largest bbox diagonal
static size_t loft_border_index(const std::vector<Polyline>& polylines) {
    size_t border_idx = 0;
    double max_diag = 0.0;
    for (size_t i = 0; i < polylines.size(); ++i) {
        const std::vector<Point> pts = polylines[i].get_points();
        if (pts.empty()) continue;
        double minx = pts[0][0], miny = pts[0][1], minz = pts[0][2];
        double maxx = minx, maxy = miny, maxz = minz;
        for (const Point& p : pts) {
            minx = std::min(minx, p[0]); maxx = std::max(maxx, p[0]);
            miny = std::min(miny, p[1]); maxy = std::max(maxy, p[1]);
            minz = std::min(minz, p[2]); maxz = std::max(maxz, p[2]);
        }
        const double dx = maxx - minx, dy = maxy - miny, dz = maxz - minz;
        const double diag = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (diag > max_diag) { max_diag = diag; border_idx = i; }
    }
    return border_idx;
}

/// Projection frame of the border polyline, y flipped so z points from bottom to top
static LoftFrame loft_frame(const Polyline& bottom, const Polyline& top) {
    Point origin;
    Vector xaxis, yaxis, zaxis;
    bottom.get_average_plane(origin, xaxis, yaxis, zaxis);
    const Point c0 = bottom.center();
    const Point c1 = top.center();
    const Vector bottom_to_top(c1[0]-c0[0], c1[1]-c0[1], c1[2]-c0[2]);
    if (zaxis.dot(bottom_to_top) < 0)
        yaxis = Vector(-yaxis[0], -yaxis[1], -yaxis[2]);
    return {origin, xaxis, yaxis};
}

/// Vertex keys of pts; consecutive same-position points share one key
static std::vector<size_t> loft_add_vkeys(Mesh& mesh, const std::vector<Point>& pts) {
    std::vector<size_t> keys;
    keys.reserve(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
        if (i > 0) {
            const Point& prev = pts[i-1];
            const Point& curr = pts[i];
            const double dx = curr[0]-prev[0], dy = curr[1]-prev[1], dz = curr[2]-prev[2];
            if (dx*dx + dy*dy + dz*dz < 1e-20) {
                keys.push_back(keys.back());
                continue;
            }
        }
        keys.push_back(mesh.add_vertex(pts[i]));
    }
    return keys;
}

/// Split triangle j, which spans corners a and c, at boundary vertex b
static void loft_split_triangle(std::vector<std::array<size_t,3>>& tris, size_t j, size_t a, size_t c, size_t b) {
    const std::array<size_t,3> ft = tris[j];
    std::array<size_t,3> t1, t2;
    if ((ft[0]==a||ft[0]==c)&&(ft[1]==a||ft[1]==c))
        { t1={ft[0],b,ft[2]}; t2={b,ft[1],ft[2]}; }
    else if ((ft[1]==a||ft[1]==c)&&(ft[2]==a||ft[2]==c))
        { t1={ft[0],ft[1],b}; t2={ft[0],b,ft[2]}; }
    else
        { t1={ft[0],ft[1],b}; t2={b,ft[1],ft[2]}; }
    tris[j] = t1;
    tris.push_back(t2);
}

/// Insert boundary vertices the CDT skipped as collinear, one per pass
static void loft_fix_collinear(std::vector<std::array<size_t,3>>& tris, const std::vector<size_t>& fvkeys) {
    const size_t n = fvkeys.size();
    for (size_t pass = 0; pass < n; ++pass) {
        std::set<size_t> used;
        for (const std::array<size_t,3>& t : tris)
            for (size_t v : t) used.insert(v);
        bool changed = false;
        for (size_t k = 0; k < n && !changed; ++k) {
            const size_t b = fvkeys[k];
            if (used.count(b)) continue;
            const size_t a = fvkeys[(k+n-1)%n], c = fvkeys[(k+1)%n];
            for (size_t j = 0; j < tris.size(); ++j) {
                const bool has_a = (tris[j][0]==a||tris[j][1]==a||tris[j][2]==a);
                const bool has_c = (tris[j][0]==c||tris[j][1]==c||tris[j][2]==c);
                if (!has_a || !has_c) continue;
                loft_split_triangle(tris, j, a, c, b);
                changed = true;
                break;
            }
        }
        if (!changed) return;
    }
}

/// Drop triangles with zero area in the projected integer grid
static void loft_drop_degenerate(std::vector<std::array<size_t,3>>& tris, const Mesh& mesh, const LoftFrame& frame) {
    const double sc = 1e6;
    std::vector<std::array<size_t,3>> kept;
    kept.reserve(tris.size());
    for (const std::array<size_t,3>& t : tris) {
        const auto [u0, v0] = loft_project(frame, mesh.vertex.at(t[0]).position());
        const auto [u1, v1] = loft_project(frame, mesh.vertex.at(t[1]).position());
        const auto [u2, v2] = loft_project(frame, mesh.vertex.at(t[2]).position());
        const int64_t iu0 = std::llround(u0*sc), iv0 = std::llround(v0*sc);
        const int64_t iu1 = std::llround(u1*sc), iv1 = std::llround(v1*sc);
        const int64_t iu2 = std::llround(u2*sc), iv2 = std::llround(v2*sc);
        if ((iu1-iu0)*(iv2-iv0)-(iv1-iv0)*(iu2-iu0) != 0) kept.push_back(t);
    }
    tris = kept;
}

/// One n-gon cap with stored CDT triangulation and hole rings; reversed for the bottom
static void loft_cap(
    Mesh& mesh,
    const LoftFrame& frame,
    const std::vector<LoftRing>& rings,
    const std::vector<Point>& pts,
    const std::vector<size_t>& vkeys,
    bool reverse,
    bool fix_collinear
) {
    std::vector<std::pair<double,double>> border_2d;
    std::vector<size_t> outer;
    for (size_t i = 0; i < rings[0].n; ++i) {
        const size_t vi = rings[0].off + i;
        if (!outer.empty() && vkeys[vi] == vkeys[outer.back()]) continue;
        border_2d.push_back(loft_project(frame, pts[vi]));
        outer.push_back(vi);
    }
    std::vector<size_t> flat = outer;
    std::vector<std::vector<std::pair<double,double>>> holes_2d;
    std::vector<std::vector<size_t>> hole_rings;
    for (size_t h = 1; h < rings.size(); ++h) {
        std::vector<std::pair<double,double>> hole;
        std::vector<size_t> ring;
        for (size_t i = rings[h].off; i < rings[h].off + rings[h].n; ++i) {
            hole.push_back(loft_project(frame, pts[i]));
            flat.push_back(i);
            ring.push_back(vkeys[i]);
        }
        holes_2d.push_back(std::move(hole));
        hole_rings.push_back(std::move(ring));
    }
    const std::vector<std::array<int,3>> tris = cdt_triangulate(border_2d, holes_2d);
    std::vector<size_t> fvkeys;
    fvkeys.reserve(outer.size());
    for (size_t i = 0; i < outer.size(); ++i)
        fvkeys.push_back(vkeys[outer[reverse ? outer.size() - 1 - i : i]]);
    const std::optional<size_t> fk = mesh.add_face(fvkeys);
    if (!fk) return;
    std::vector<std::array<size_t,3>> tri_list;
    tri_list.reserve(tris.size());
    for (const std::array<int,3>& t : tris) {
        if (reverse) tri_list.push_back({vkeys[flat[t[0]]], vkeys[flat[t[2]]], vkeys[flat[t[1]]]});
        else tri_list.push_back({vkeys[flat[t[0]]], vkeys[flat[t[1]]], vkeys[flat[t[2]]]});
    }
    if (fix_collinear) {
        loft_fix_collinear(tri_list, fvkeys);
        loft_drop_degenerate(tri_list, mesh, frame);
    }
    mesh.set_face_triangulation(*fk, tri_list);
    if (!hole_rings.empty() && !tri_list.empty())
        mesh.set_face_holes(*fk, hole_rings);
}

/// Squared 2D length of edge i of a ring
static double loft_edge_sq_2d(const LoftFrame& frame, const std::vector<Point>& pts, size_t i) {
    const size_t j = (i + 1) % pts.size();
    const auto [xi, yi] = loft_project(frame, pts[i]);
    const auto [xj, yj] = loft_project(frame, pts[j]);
    const double dx = xj - xi, dy = yj - yi;
    return dx*dx + dy*dy;
}

/// Start offsets (ia, ib): the longest bottom edge, and the top offset that minimizes the projected gap
static std::pair<size_t, size_t> loft_wall_start(const LoftFrame& frame, const std::vector<Point>& bpts, const std::vector<Point>& tpts) {
    const size_t bot_n = bpts.size(), top_n = tpts.size();
    size_t ia = 0, ib = 0;
    double max_b = 0;
    for (size_t k = 0; k < bot_n; ++k) {
        const double v = loft_edge_sq_2d(frame, bpts, k);
        if (v > max_b) { max_b = v; ia = k; }
    }
    if (bot_n != top_n) return {ia, ib};
    double min_total = std::numeric_limits<double>::max();
    for (size_t cand = 0; cand < top_n; ++cand) {
        double total = 0.0;
        for (size_t k = 0; k < bot_n; ++k) {
            const auto [xb, yb] = loft_project(frame, bpts[(ia+k)%bot_n]);
            const auto [xt, yt] = loft_project(frame, tpts[(cand+k)%top_n]);
            total += (xt-xb)*(xt-xb) + (yt-yb)*(yt-yb);
        }
        if (total < min_total) { min_total = total; ib = cand; }
    }
    return {ia, ib};
}

static bool loft_same_point(const Point& a, const Point& b) {
    return std::abs(a[0]-b[0]) < 1e-10 && std::abs(a[1]-b[1]) < 1e-10 && std::abs(a[2]-b[2]) < 1e-10;
}

/// Quad walls for equal counts; a collapsed bottom or top edge gives a triangle
static void loft_walls_quads(
    Mesh& mesh,
    const LoftPoly& poly,
    size_t ia,
    size_t ib,
    const std::vector<Point>& bpts,
    const std::vector<Point>& tpts,
    const std::vector<size_t>& bot_vkeys,
    const std::vector<size_t>& top_vkeys
) {
    const size_t bot_n = poly.bot.n, top_n = poly.top.n;
    for (size_t k = 0; k < bot_n; ++k) {
        const size_t cb = poly.bot.off+(ia+k)%bot_n, ct = poly.top.off+(ib+k)%top_n;
        const size_t nb = poly.bot.off+(ia+k+1)%bot_n, nt = poly.top.off+(ib+k+1)%top_n;
        const bool bot_col = loft_same_point(bpts[(ia+k)%bot_n], bpts[(ia+k+1)%bot_n]);
        const bool top_col = loft_same_point(tpts[(ib+k)%top_n], tpts[(ib+k+1)%top_n]);
        if (bot_col && top_col) continue;
        else if (bot_col) mesh.add_face({bot_vkeys[cb], top_vkeys[nt], top_vkeys[ct]});
        else if (top_col) mesh.add_face({bot_vkeys[cb], bot_vkeys[nb], top_vkeys[ct]});
        else mesh.add_face({bot_vkeys[cb], bot_vkeys[nb], top_vkeys[nt], top_vkeys[ct]});
    }
}

/// Normalized arc lengths of a ring starting at offset start
static std::vector<double> loft_arcs(const std::vector<Point>& pts, size_t start) {
    const size_t n = pts.size();
    std::vector<double> arcs(n + 1, 0.0);
    for (size_t k = 0; k < n; ++k) {
        const size_t i = (start+k)%n, j = (start+k+1)%n;
        const double dx = pts[j][0]-pts[i][0], dy = pts[j][1]-pts[i][1], dz = pts[j][2]-pts[i][2];
        arcs[k+1] = arcs[k] + std::sqrt(dx*dx+dy*dy+dz*dz);
    }
    const double inv = arcs[n] > 0 ? 1.0/arcs[n] : 1.0;
    for (double& a : arcs) a *= inv;
    return arcs;
}

/// Zipper walls for unequal counts: quads where arc lengths meet, triangles elsewhere
static void loft_walls_zipper(
    Mesh& mesh,
    const LoftPoly& poly,
    size_t ia,
    size_t ib,
    const std::vector<Point>& bpts,
    const std::vector<Point>& tpts,
    const std::vector<size_t>& bot_vkeys,
    const std::vector<size_t>& top_vkeys
) {
    const size_t bot_n = poly.bot.n, top_n = poly.top.n;
    const std::vector<double> b_arcs = loft_arcs(bpts, ia);
    const std::vector<double> t_arcs = loft_arcs(tpts, ib);
    size_t bi = 0, ti = 0;
    while (bi < bot_n || ti < top_n) {
        const size_t cb = poly.bot.off+(ia+bi)%bot_n, ct = poly.top.off+(ib+ti)%top_n;
        const size_t nb = poly.bot.off+(ia+bi+1)%bot_n, nt = poly.top.off+(ib+ti+1)%top_n;
        if (bi >= bot_n) {
            mesh.add_face({bot_vkeys[cb], top_vkeys[ct], top_vkeys[nt]}); ++ti;
        } else if (ti >= top_n) {
            mesh.add_face({bot_vkeys[cb], bot_vkeys[nb], top_vkeys[ct]}); ++bi;
        } else if (std::abs(b_arcs[bi+1] - t_arcs[ti+1]) < 1e-9) {
            mesh.add_face({bot_vkeys[cb], bot_vkeys[nb], top_vkeys[nt], top_vkeys[ct]}); ++bi; ++ti;
        } else if (b_arcs[bi+1] < t_arcs[ti+1]) {
            mesh.add_face({bot_vkeys[cb], bot_vkeys[nb], top_vkeys[ct]}); ++bi;
        } else {
            mesh.add_face({bot_vkeys[cb], top_vkeys[ct], top_vkeys[nt]}); ++ti;
        }
    }
}

static void loft_walls(
    Mesh& mesh,
    const LoftFrame& frame,
    const LoftPoly& poly,
    const std::vector<Point>& all_bot,
    const std::vector<Point>& all_top,
    const std::vector<size_t>& bot_vkeys,
    const std::vector<size_t>& top_vkeys
) {
    const std::vector<Point> bpts(all_bot.begin()+poly.bot.off, all_bot.begin()+poly.bot.off+poly.bot.n);
    const std::vector<Point> tpts(all_top.begin()+poly.top.off, all_top.begin()+poly.top.off+poly.top.n);
    const auto [ia, ib] = loft_wall_start(frame, bpts, tpts);
    if (poly.bot.n == poly.top.n)
        loft_walls_quads(mesh, poly, ia, ib, bpts, tpts, bot_vkeys, top_vkeys);
    else
        loft_walls_zipper(mesh, poly, ia, ib, bpts, tpts, bot_vkeys, top_vkeys);
}

Mesh Mesh::loft(const std::vector<Polyline>& polylines0, const std::vector<Polyline>& polylines1, bool cap, bool fix_collinear) {
    if (polylines0.empty() || polylines1.empty()) return Mesh();
    if (polylines0.size() != polylines1.size()) return Mesh();
    const size_t border_idx = loft_border_index(polylines0);
    const LoftFrame frame = loft_frame(polylines0[border_idx], polylines1[border_idx]);
    std::vector<size_t> order;
    order.push_back(border_idx);
    for (size_t i = 0; i < polylines0.size(); ++i)
        if (i != border_idx) order.push_back(i);
    std::vector<LoftPoly> polys;
    std::vector<Point> all_bot;
    std::vector<Point> all_top;
    for (size_t oi = 0; oi < order.size(); ++oi) {
        std::vector<Point> bot = loft_open_points(polylines0[order[oi]]);
        std::vector<Point> top = loft_open_points(polylines1[order[oi]]);
        const double area = loft_signed_area(frame, bot);
        if (oi == 0 ? (area < 0) : (area > 0)) {
            std::reverse(bot.begin(), bot.end());
            std::reverse(top.begin(), top.end());
        }
        polys.push_back({{all_bot.size(), bot.size()}, {all_top.size(), top.size()}});
        for (const Point& p : bot) all_bot.push_back(p);
        for (const Point& p : top) all_top.push_back(p);
    }
    Mesh mesh;
    const std::vector<size_t> bot_vkeys = loft_add_vkeys(mesh, all_bot);
    const std::vector<size_t> top_vkeys = loft_add_vkeys(mesh, all_top);
    if (cap) {
        std::vector<LoftRing> bot_rings, top_rings;
        for (const LoftPoly& poly : polys) {
            bot_rings.push_back(poly.bot);
            top_rings.push_back(poly.top);
        }
        loft_cap(mesh, frame, bot_rings, all_bot, bot_vkeys, true, fix_collinear);
        loft_cap(mesh, frame, top_rings, all_top, top_vkeys, false, fix_collinear);
    }
    for (const LoftPoly& poly : polys)
        loft_walls(mesh, frame, poly, all_bot, all_top, bot_vkeys, top_vkeys);
    return mesh;
}

std::vector<Mesh> Mesh::from_polygon_with_holes_many(
    const std::vector<std::vector<std::vector<Point>>>& inputs,
    bool sort_by_bbox, bool parallel)
{
    std::vector<Mesh> results(inputs.size());
    const std::function<void(size_t)> fn = [&](size_t i) { results[i] = from_polygon_with_holes(inputs[i], sort_by_bbox); };
    if (parallel && inputs.size() > 1) parallel_for(inputs.size(), fn);
    else for (size_t i = 0; i < inputs.size(); ++i) fn(i);
    return results;
}

std::vector<Mesh> Mesh::loft_many(
    const std::vector<std::pair<std::vector<Polyline>, std::vector<Polyline>>>& pairs,
    bool cap, bool parallel, bool fix_collinear)
{
    std::vector<Mesh> results(pairs.size());
    const std::function<void(size_t)> fn = [&](size_t i) { results[i] = loft(pairs[i].first, pairs[i].second, cap, fix_collinear); };
    if (parallel && pairs.size() > 1) parallel_for(pairs.size(), fn);
    else for (size_t i = 0; i < pairs.size(); ++i) fn(i);
    return results;
}

// ═══════════════════════════════════════════════════════════════════════════
// Loft panels
// ═══════════════════════════════════════════════════════════════════════════

/// Drop ring points collinear with their neighbors, until none is left
static void lp_merge_collinear(std::vector<Point>& pts, std::vector<size_t>& vkeys) {
    const double tol = Tolerance::APPROXIMATION;
    const double zt2 = Tolerance::ZERO_TOLERANCE * Tolerance::ZERO_TOLERANCE;
    const size_t bound = pts.size();
    for (size_t pass = 0; pass < bound; ++pass) {
        const size_t m = pts.size();
        if (m < 3) return;
        bool changed = false;
        std::vector<Point> np;
        std::vector<size_t> nk;
        for (size_t i = 0; i < m; i++) {
            const size_t p = (i + m - 1) % m, nx = (i + 1) % m;
            const double ax = pts[i][0]-pts[p][0], ay = pts[i][1]-pts[p][1], az = pts[i][2]-pts[p][2];
            const double bx = pts[nx][0]-pts[i][0], by = pts[nx][1]-pts[i][1], bz = pts[nx][2]-pts[i][2];
            const double cx = ay*bz-az*by, cy = az*bx-ax*bz, cz = ax*by-ay*bx;
            const double a2 = ax*ax+ay*ay+az*az, b2 = bx*bx+by*by+bz*bz;
            if (a2 < zt2 || b2 < zt2 || cx*cx+cy*cy+cz*cz < tol*tol*a2*b2) changed = true;
            else { np.push_back(pts[i]); nk.push_back(vkeys[i]); }
        }
        pts = np;
        vkeys = nk;
        if (!changed) return;
    }
}

/// Drop ring points closer than a thousandth of the longest edge to their predecessor
static void lp_merge_close(std::vector<Point>& pts, std::vector<size_t>& vkeys) {
    const size_t sz = pts.size();
    double max_edge = 0;
    for (size_t i = 0; i < sz; i++)
        max_edge = std::max(max_edge, pts[i].distance(pts[(i+1)%sz]));
    const double stol = max_edge * 0.001;
    std::vector<Point> tp;
    std::vector<size_t> tk;
    for (size_t i = 0; i < sz; i++) {
        if (tp.empty() || tp.back().distance(pts[i]) > stol) {
            tp.push_back(pts[i]);
            tk.push_back(vkeys[i]);
        }
    }
    while (tp.size() >= 3 && tp.back().distance(tp.front()) <= stol) {
        tp.pop_back();
        tk.pop_back();
    }
    if (tp.size() < 3) return;
    pts = tp;
    vkeys = tk;
}

static Point lp_offset_toward(const Point& p, double cx, double cy, double cz, double gap) {
    double dx = cx-p[0], dy = cy-p[1], dz = cz-p[2];
    const double len = std::sqrt(dx*dx+dy*dy+dz*dz);
    if (len > 1e-10) { dx *= gap/len; dy *= gap/len; dz *= gap/len; }
    return Point(p[0]+dx, p[1]+dy, p[2]+dz);
}

static Point lp_face_centroid(const Mesh& m, size_t fk) {
    const std::vector<size_t> vkeys = *m.face_vertices(fk);
    double cx = 0, cy = 0, cz = 0;
    for (size_t vk : vkeys) {
        const Point p = *m.vertex_point(vk);
        cx += p[0]; cy += p[1]; cz += p[2];
    }
    return Point(cx/vkeys.size(), cy/vkeys.size(), cz/vkeys.size());
}

/// Greedy top/bottom face pairs by centroid distance, sorted by key
static std::vector<std::pair<size_t,size_t>> lp_match_faces(const Mesh& top_mesh, const Mesh& bot_mesh) {
    const std::vector<size_t> tfks = top_mesh.faces();
    const std::vector<size_t> bfks = bot_mesh.faces();
    std::vector<std::tuple<double, size_t, size_t>> dists;
    dists.reserve(tfks.size() * bfks.size());
    for (size_t ti = 0; ti < tfks.size(); ti++)
        for (size_t bi = 0; bi < bfks.size(); bi++)
            dists.push_back({lp_face_centroid(top_mesh, tfks[ti]).distance(lp_face_centroid(bot_mesh, bfks[bi])), ti, bi});
    std::sort(dists.begin(), dists.end());
    std::vector<bool> top_used(tfks.size(), false), bot_used(bfks.size(), false);
    std::vector<std::pair<size_t,size_t>> face_match;
    for (const auto& [d, ti, bi] : dists) {
        if (top_used[ti] || bot_used[bi]) continue;
        face_match.push_back({tfks[ti], bfks[bi]});
        top_used[ti] = true;
        bot_used[bi] = true;
    }
    std::sort(face_match.begin(), face_match.end());
    return face_match;
}

/// Reverse top and bottom rings so the top normal points from bottom to top and the bottom normal away
static void lp_orient_rings(std::vector<Point>& top_pts, std::vector<size_t>& top_vkeys, std::vector<Point>& bot_pts, std::vector<size_t>& bot_vkeys) {
    const Point tc = ring_centroid(top_pts);
    const Point bc = ring_centroid(bot_pts);
    Vector axis(tc[0]-bc[0], tc[1]-bc[1], tc[2]-bc[2]);
    if (!axis.normalize_self()) return;
    if (newell_normal(top_pts).dot(axis) < 0) {
        std::reverse(top_pts.begin(), top_pts.end());
        std::reverse(top_vkeys.begin(), top_vkeys.end());
    }
    if (newell_normal(bot_pts).dot(axis) > 0) {
        std::reverse(bot_pts.begin(), bot_pts.end());
        std::reverse(bot_vkeys.begin(), bot_vkeys.end());
    }
}

/// Cap face over local keys with its planar CDT stored
static std::optional<size_t> lp_add_cap(Mesh& mesh, const std::vector<size_t>& cap, const std::vector<Point>& pts) {
    const std::optional<size_t> fk = mesh.add_face(cap);
    if (!fk || cap.size() < 3) return fk;
    const std::vector<std::array<int,3>> tris = planar_cdt(pts);
    if (tris.empty()) return fk;
    std::vector<std::array<size_t,3>> tri_list;
    tri_list.reserve(tris.size());
    for (const std::array<int,3>& t : tris)
        tri_list.push_back({cap[t[0]], cap[t[1]], cap[t[2]]});
    mesh.set_face_triangulation(*fk, std::move(tri_list));
    return fk;
}

/// Edge midpoints of a ring
static std::vector<Point> lp_midpoints(const std::vector<Point>& pts) {
    const size_t n = pts.size();
    std::vector<Point> mids(n);
    for (size_t i = 0; i < n; i++)
        mids[i] = Point((pts[i][0]+pts[(i+1)%n][0])*0.5, (pts[i][1]+pts[(i+1)%n][1])*0.5, (pts[i][2]+pts[(i+1)%n][2])*0.5);
    return mids;
}

/// Index of the point in pts nearest to p
static size_t lp_nearest(const Point& p, const std::vector<Point>& pts) {
    double best_d = std::numeric_limits<double>::max();
    size_t best = 0;
    for (size_t i = 0; i < pts.size(); i++) {
        const double d = p.distance(pts[i]);
        if (d < best_d) { best_d = d; best = i; }
    }
    return best;
}

/// Quad wall over matched edge j of the bottom and ti of the top, inset by edge_gap
static std::optional<size_t> lp_add_quad(LoftPanel& panel, size_t b0, size_t b1, size_t t0, size_t t1, double edge_gap) {
    if (edge_gap <= 0.0) return panel.mesh.add_face({b0, t1, t0, b1});
    const Point pb0 = *panel.mesh.vertex_point(b0);
    const Point pb1 = *panel.mesh.vertex_point(b1);
    const Point pt0 = *panel.mesh.vertex_point(t0);
    const Point pt1 = *panel.mesh.vertex_point(t1);
    const double cx = (pb0[0]+pb1[0]+pt0[0]+pt1[0])*0.25;
    const double cy = (pb0[1]+pb1[1]+pt0[1]+pt1[1])*0.25;
    const double cz = (pb0[2]+pb1[2]+pt0[2]+pt1[2])*0.25;
    const size_t nb0 = panel.mesh.add_vertex(lp_offset_toward(pb0, cx, cy, cz, edge_gap));
    const size_t nb1 = panel.mesh.add_vertex(lp_offset_toward(pb1, cx, cy, cz, edge_gap));
    return panel.mesh.add_face({nb0, t1, t0, nb1});
}

/// A triangle from every unmatched top edge to the nearest bottom vertex
static void lp_add_top_triangles(
    LoftPanel& panel,
    const std::vector<Point>& top_mids,
    const std::vector<size_t>& top_vkeys,
    const std::vector<Point>& bot_pts,
    const std::vector<size_t>& bot_vkeys,
    const std::vector<bool>& top_used
) {
    const size_t n = top_vkeys.size();
    for (size_t i = 0; i < n; i++) {
        if (top_used[i]) continue;
        const size_t t0 = panel.orig_top_to_local[top_vkeys[i]];
        const size_t t1 = panel.orig_top_to_local[top_vkeys[(i+1)%n]];
        const size_t bv = panel.orig_bot_to_local[bot_vkeys[lp_nearest(top_mids[i], bot_pts)]];
        const std::optional<size_t> fk = panel.mesh.add_face({t1, t0, bv});
        if (fk) {
            LoftWallFace w;
            w.face_key = *fk;
            panel.wall_faces.push_back(w);
        }
    }
}

/// Walls of one panel: a quad per mutually nearest edge pair, a triangle for every unmatched edge
static void lp_add_walls(
    LoftPanel& panel,
    const std::vector<Point>& top_pts,
    const std::vector<size_t>& top_vkeys,
    const std::vector<Point>& bot_pts,
    const std::vector<size_t>& bot_vkeys,
    double edge_gap,
    double edge_match_threshold,
    bool skip_triangles
) {
    const size_t n = top_pts.size(), m = bot_pts.size();
    const std::vector<Point> top_mids = lp_midpoints(top_pts);
    const std::vector<Point> bot_mids = lp_midpoints(bot_pts);
    std::vector<size_t> bot_to_top(m), top_to_bot(n);
    std::vector<double> bot_dist(m);
    for (size_t j = 0; j < m; j++) {
        bot_to_top[j] = lp_nearest(bot_mids[j], top_mids);
        bot_dist[j] = bot_mids[j].distance(top_mids[bot_to_top[j]]);
    }
    for (size_t i = 0; i < n; i++)
        top_to_bot[i] = lp_nearest(top_mids[i], bot_mids);
    double avg = 0;
    for (size_t j = 0; j < m; j++) avg += bot_dist[j];
    const double threshold = avg / m * edge_match_threshold;
    std::vector<bool> top_used(n, false);
    for (size_t j = 0; j < m; j++) {
        const size_t b0 = panel.orig_bot_to_local[bot_vkeys[j]];
        const size_t b1 = panel.orig_bot_to_local[bot_vkeys[(j+1)%m]];
        const size_t ti = bot_to_top[j];
        if (bot_dist[j] <= threshold && top_to_bot[ti] == j) {
            const size_t t0 = panel.orig_top_to_local[top_vkeys[ti]];
            const size_t t1 = panel.orig_top_to_local[top_vkeys[(ti+1)%n]];
            const std::optional<size_t> fk = lp_add_quad(panel, b0, b1, t0, t1, edge_gap);
            if (fk) {
                LoftWallFace w;
                w.face_key = *fk;
                w.is_quad = true;
                w.top_v0 = top_vkeys[ti];
                w.top_v1 = top_vkeys[(ti+1)%n];
                w.bot_v0 = bot_vkeys[(j+1)%m];
                w.bot_v1 = bot_vkeys[j];
                panel.wall_faces.push_back(w);
            }
            top_used[ti] = true;
        } else if (!skip_triangles) {
            const size_t tv = panel.orig_top_to_local[top_vkeys[lp_nearest(bot_mids[j], top_pts)]];
            const std::optional<size_t> fk = panel.mesh.add_face({b0, tv, b1});
            if (fk) {
                LoftWallFace w;
                w.face_key = *fk;
                panel.wall_faces.push_back(w);
            }
        }
    }
    if (!skip_triangles) lp_add_top_triangles(panel, top_mids, top_vkeys, bot_pts, bot_vkeys, top_used);
}

/// One panel between matched faces tfk of the top mesh and bfk of the bottom mesh
static LoftPanel lp_build_panel(
    const Mesh& top_mesh,
    const Mesh& bot_mesh,
    size_t tfk,
    size_t bfk,
    double edge_gap,
    double edge_match_threshold,
    bool add_caps,
    bool skip_triangles
) {
    LoftPanel panel;
    std::vector<size_t> top_vkeys = *top_mesh.face_vertices(tfk);
    std::vector<size_t> bot_vkeys = *bot_mesh.face_vertices(bfk);
    std::vector<Point> top_pts, bot_pts;
    for (size_t vk : top_vkeys) top_pts.push_back(*top_mesh.vertex_point(vk));
    for (size_t vk : bot_vkeys) bot_pts.push_back(*bot_mesh.vertex_point(vk));
    lp_merge_collinear(top_pts, top_vkeys);
    lp_merge_collinear(bot_pts, bot_vkeys);
    lp_merge_close(top_pts, top_vkeys);
    lp_orient_rings(top_pts, top_vkeys, bot_pts, bot_vkeys);
    for (size_t i = 0; i < top_pts.size(); i++) {
        const size_t lk = panel.mesh.add_vertex(top_pts[i]);
        panel.orig_top_to_local[top_vkeys[i]] = lk;
        panel.top_vertices.push_back(lk);
    }
    for (size_t j = 0; j < bot_pts.size(); j++) {
        const size_t lk = panel.mesh.add_vertex(bot_pts[j]);
        panel.orig_bot_to_local[bot_vkeys[j]] = lk;
        panel.bot_vertices.push_back(lk);
    }
    if (add_caps)
        panel.top_face_key = lp_add_cap(panel.mesh, panel.top_vertices, top_pts);
    lp_add_walls(panel, top_pts, top_vkeys, bot_pts, bot_vkeys, edge_gap, edge_match_threshold, skip_triangles);
    if (add_caps)
        panel.bot_face_key = lp_add_cap(panel.mesh, panel.bot_vertices, bot_pts);
    std::map<size_t,size_t> fkey_to_idx;
    size_t fi = 0;
    for (const auto& [fk, _] : panel.mesh.face)
        fkey_to_idx[fk] = fi++;
    for (LoftWallFace& w : panel.wall_faces) {
        w.face_index = fkey_to_idx[w.face_key];
        panel.face_roles[w.face_key] = w.is_quad ? LoftFaceRole::QuadWall : LoftFaceRole::TriWall;
    }
    if (panel.top_face_key) panel.face_roles[*panel.top_face_key] = LoftFaceRole::TopCap;
    if (panel.bot_face_key) panel.face_roles[*panel.bot_face_key] = LoftFaceRole::BotCap;
    return panel;
}

/// Quad walls of different panels that share a top edge, once per pair
static std::vector<LoftAdjPair> lp_adjacency(const std::vector<LoftPanel>& panels) {
    std::map<std::pair<size_t,size_t>, std::pair<size_t,size_t>> edge_to_wall;
    for (size_t pi = 0; pi < panels.size(); pi++)
        for (size_t wi = 0; wi < panels[pi].wall_faces.size(); wi++) {
            const LoftWallFace& w = panels[pi].wall_faces[wi];
            if (w.is_quad) edge_to_wall[{w.top_v0, w.top_v1}] = {pi, wi};
        }
    std::vector<LoftAdjPair> adjacency;
    for (size_t pi = 0; pi < panels.size(); pi++)
        for (size_t wi = 0; wi < panels[pi].wall_faces.size(); wi++) {
            const LoftWallFace& w = panels[pi].wall_faces[wi];
            if (!w.is_quad) continue;
            const auto it = edge_to_wall.find({w.top_v1, w.top_v0});
            if (it != edge_to_wall.end() && it->second.first > pi)
                adjacency.push_back({pi, wi, it->second.first, it->second.second});
        }
    return adjacency;
}

/// One face per panel, from its local top or bottom ring
static Mesh lp_ordered_mesh(const std::vector<LoftPanel>& panels, bool top) {
    Mesh ordered;
    for (size_t i = 0; i < panels.size(); i++) {
        const std::vector<size_t>& ring = top ? panels[i].top_vertices : panels[i].bot_vertices;
        std::vector<size_t> vks;
        vks.reserve(ring.size());
        for (size_t lk : ring) vks.push_back(ordered.add_vertex(*panels[i].mesh.vertex_point(lk)));
        ordered.add_face(vks, i);
    }
    return ordered;
}

LoftResult Mesh::loft_panels(
    const std::vector<std::vector<Point>>& top_polygons,
    const std::vector<std::vector<Point>>& bot_polygons,
    double merge_precision,
    double edge_gap,
    double edge_match_threshold,
    bool   add_caps,
    bool   skip_triangles)
{
    const Mesh top_mesh = Mesh::from_polylines(top_polygons, merge_precision);
    const Mesh bot_mesh = Mesh::from_polylines(bot_polygons, merge_precision);
    const std::vector<std::pair<size_t,size_t>> face_match = lp_match_faces(top_mesh, bot_mesh);
    std::vector<LoftPanel> panels;
    panels.reserve(face_match.size());
    for (const auto& [tfk, bfk] : face_match)
        panels.push_back(lp_build_panel(top_mesh, bot_mesh, tfk, bfk, edge_gap, edge_match_threshold, add_caps, skip_triangles));
    std::vector<LoftAdjPair> adjacency = lp_adjacency(panels);
    Mesh top_ordered = lp_ordered_mesh(panels, true);
    Mesh bot_ordered = lp_ordered_mesh(panels, false);
    return {std::move(panels), std::move(adjacency), std::move(top_ordered), std::move(bot_ordered)};
}

Mesh Mesh::create_box(double x, double y, double z) {
    const double hx = x * 0.5, hy = y * 0.5, hz = z * 0.5;
    const std::vector<Point> vertices = {
        Point(-hx, -hy, -hz),
        Point( hx, -hy, -hz),
        Point( hx,  hy, -hz),
        Point(-hx,  hy, -hz),
        Point(-hx, -hy,  hz),
        Point( hx, -hy,  hz),
        Point( hx,  hy,  hz),
        Point(-hx,  hy,  hz),
    };
    const std::vector<std::vector<size_t>> faces = {
        {0, 3, 2, 1},
        {4, 5, 6, 7},
        {0, 1, 5, 4},
        {2, 3, 7, 6},
        {0, 4, 7, 3},
        {1, 2, 6, 5},
    };
    return from_vertices_and_faces(vertices, faces);
}

Mesh Mesh::create_dodecahedron(double edge) {
    const double phi = (1.0 + std::sqrt(5.0)) / 2.0;
    const double ip = 1.0 / phi;
    const double s = edge / (2.0 * ip);
    const Point verts[20] = {
        Point(s, s, s),
        Point(s, s, -s),
        Point(s, -s, s),
        Point(s, -s, -s),
        Point(-s, s, s),
        Point(-s, s, -s),
        Point(-s, -s, s),
        Point(-s, -s, -s),
        Point(0, s*ip, s*phi),
        Point(0, s*ip, -s*phi),
        Point(0, -s*ip, s*phi),
        Point(0, -s*ip, -s*phi),
        Point(s*ip, s*phi, 0),
        Point(s*ip, -s*phi, 0),
        Point(-s*ip, s*phi, 0),
        Point(-s*ip, -s*phi, 0),
        Point(s*phi, 0, s*ip),
        Point(s*phi, 0, -s*ip),
        Point(-s*phi, 0, s*ip),
        Point(-s*phi, 0, -s*ip),
    };
    const int idx[12][5] = {
        {0, 8, 10, 2, 16}, {0, 16, 17, 1, 12}, {0, 12, 14, 4, 8},
        {1, 17, 3, 11, 9}, {1, 9, 5, 14, 12}, {2, 10, 6, 15, 13},
        {2, 13, 3, 17, 16}, {3, 13, 15, 7, 11}, {4, 14, 5, 19, 18},
        {4, 18, 6, 10, 8}, {5, 9, 11, 7, 19}, {6, 18, 19, 7, 15},
    };
    std::vector<std::vector<Point>> faces;
    faces.reserve(12);
    for (const auto& f : idx)
        faces.push_back({verts[f[0]], verts[f[1]], verts[f[2]], verts[f[3]], verts[f[4]]});
    return from_polylines(faces, 1e-6);
}

/// Number of points of a polyline without its closing duplicate
static size_t pairs_open_count(const Polyline& pl) {
    const size_t n = pl.point_count();
    if (n > 1 && pl.is_closed()) return n - 1;
    return n;
}

/// Open polyline with every coordinate divided by scale
static Polyline pairs_scaled_open(const Polyline& src, double scale) {
    const size_t limit = pairs_open_count(src);
    std::vector<Point> pts;
    pts.reserve(limit);
    for (size_t j = 0; j < limit; ++j) {
        const Point p = src.get_point(j);
        pts.push_back(Point(p[0] / scale, p[1] / scale, p[2] / scale));
    }
    return Polyline(pts);
}

Mesh Mesh::from_polyline_pairs(const std::vector<Polyline>& pairs, double scale) {
    if (pairs.empty() || pairs.size() % 2 != 0) return Mesh();
    for (size_t i = 0; i < pairs.size(); i += 2) {
        const size_t a = pairs_open_count(pairs[i]);
        const size_t b = pairs_open_count(pairs[i + 1]);
        if (a != b || a < 3) return Mesh();
    }
    std::vector<Polyline> top_polys, bot_polys;
    top_polys.reserve(pairs.size() / 2);
    bot_polys.reserve(pairs.size() / 2);
    for (size_t i = 0; i < pairs.size(); i += 2) {
        top_polys.push_back(pairs_scaled_open(pairs[i], scale));
        bot_polys.push_back(pairs_scaled_open(pairs[i + 1], scale));
    }
    return loft(top_polys, bot_polys, true);
}

void Mesh::from_polyline_pairs_vnf(
    const std::vector<Polyline>& pairs,
    std::vector<double>& out_vertices,
    std::vector<double>& out_normals,
    std::vector<int>& out_triangles,
    double scale)
{
    const Mesh m = from_polyline_pairs(pairs, scale);
    if (m.is_empty()) return;
    const std::map<size_t, Vector> face_nrms = m.face_normals();
    for (size_t fk : m.faces()) {
        const std::optional<std::vector<Point>> fpts = m.face_points(fk);
        if (!fpts || fpts->size() < 3) continue;
        Vector nrm(0.0, 0.0, 1.0);
        const auto it = face_nrms.find(fk);
        if (it != face_nrms.end()) nrm = it->second;
        for (size_t i = 1; i + 1 < fpts->size(); ++i) {
            for (const Point* p : {&(*fpts)[0], &(*fpts)[i], &(*fpts)[i + 1]}) {
                out_triangles.push_back(static_cast<int>(out_triangles.size()));
                out_vertices.push_back((*p)[0]);
                out_vertices.push_back((*p)[1]);
                out_vertices.push_back((*p)[2]);
                out_normals.push_back(nrm[0]);
                out_normals.push_back(nrm[1]);
                out_normals.push_back(nrm[2]);
            }
        }
    }
}

Mesh Mesh::reflex_fold(const Polyline& cross_section, const Polyline& profile) {
    const size_t n_cs = cross_section.point_count();
    const size_t n_p = profile.point_count();
    std::vector<Plane> planes;
    planes.reserve(n_cs);
    for (size_t i = 0; i < n_cs; ++i) {
        Vector normal(0, 0, 1);
        if (i > 0 && i < n_cs - 1) {
            const Point ci = cross_section[i];
            const Point cp = cross_section[i - 1];
            const Point cn = cross_section[i + 1];
            const Vector v1 = Vector(cp[0]-ci[0], cp[1]-ci[1], cp[2]-ci[2]).normalized();
            const Vector v2 = Vector(cn[0]-ci[0], cn[1]-ci[1], cn[2]-ci[2]).normalized();
            normal = Vector(v1[0]+v2[0], v1[1]+v2[1], v1[2]+v2[2]);
            if (!normal.normalize_self()) normal = Vector(0, 0, 1);
        }
        planes.push_back(Plane::from_point_normal(cross_section[i], normal));
    }
    std::vector<Point> all_pts;
    all_pts.reserve(n_cs * n_p);
    for (size_t j = 0; j < n_p; ++j)
        all_pts.push_back(profile[j]);
    std::vector<std::vector<size_t>> faces;
    for (size_t i = 1; i < n_cs; ++i) {
        const Point& po = planes[i].origin();
        const Point& pp = planes[i - 1].origin();
        const Vector n1(po[0]-pp[0], po[1]-pp[1], po[2]-pp[2]);
        const Vector& n2 = planes[i].z_axis();
        const size_t row_start = all_pts.size();
        for (size_t j = 0; j < n_p; ++j) {
            const Point pvrt = all_pts[row_start - n_p + j];
            const Vector diff(po[0]-pvrt[0], po[1]-pvrt[1], po[2]-pvrt[2]);
            const double denom = n2.dot(n1);
            const double t = (std::abs(denom) > 1e-12) ? n2.dot(diff) / denom : 0.0;
            all_pts.push_back(Point(pvrt[0]+n1[0]*t, pvrt[1]+n1[1]*t, pvrt[2]+n1[2]*t));
        }
        for (size_t j = 0; j + 1 < n_p; ++j) {
            const size_t new_j = row_start + j;
            const size_t old_j = row_start - n_p + j;
            faces.push_back({new_j, old_j, old_j + 1, new_j + 1});
        }
    }
    return Mesh::from_vertices_and_faces(all_pts, faces);
}

// ═══════════════════════════════════════════════════════════════════════════
// Miter contours
// ═══════════════════════════════════════════════════════════════════════════

/// Corners whose interior angle is below max_angle_deg
static std::vector<bool> fold_chamfer_mask(const std::vector<Point>& pts, double max_angle_deg) {
    const size_t n = pts.size();
    std::vector<bool> mask(n, false);
    for (size_t i = 0; i < n; ++i) {
        const size_t prev = (i + n - 1) % n;
        const size_t next = (i + 1) % n;
        const Vector dp(pts[prev][0]-pts[i][0], pts[prev][1]-pts[i][1], pts[prev][2]-pts[i][2]);
        const Vector dn(pts[next][0]-pts[i][0], pts[next][1]-pts[i][1], pts[next][2]-pts[i][2]);
        const double lp = dp.magnitude();
        const double ln = dn.magnitude();
        if (lp < 1e-12 || ln < 1e-12) continue;
        const double cos_a = std::clamp(dp.dot(dn) / (lp * ln), -1.0, 1.0);
        mask[i] = std::acos(cos_a) * Tolerance::TO_DEGREES < max_angle_deg;
    }
    return mask;
}

/// Ring with every masked corner cut back by s, capped at a third of the shortest edge
static std::vector<Point> fold_chamfer(const std::vector<Point>& pts, double s, const std::vector<bool>& mask) {
    const size_t n = pts.size();
    if (s <= 0.0) return pts;
    double min_edge = std::numeric_limits<double>::max();
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        const Vector d(pts[j][0]-pts[i][0], pts[j][1]-pts[i][1], pts[j][2]-pts[i][2]);
        min_edge = std::min(min_edge, d.magnitude());
    }
    const double sc = std::min(s, min_edge / 3.0);
    std::vector<Point> result;
    result.reserve(2 * n);
    for (size_t i = 0; i < n; ++i) {
        if (!mask[i]) {
            result.push_back(pts[i]);
            continue;
        }
        const size_t prev = (i + n - 1) % n;
        const size_t next = (i + 1) % n;
        const Vector dp(pts[prev][0]-pts[i][0], pts[prev][1]-pts[i][1], pts[prev][2]-pts[i][2]);
        const Vector dn(pts[next][0]-pts[i][0], pts[next][1]-pts[i][1], pts[next][2]-pts[i][2]);
        const double lp = dp.magnitude();
        const double ln = dn.magnitude();
        const double sp = (lp > 1e-12) ? sc / lp : 0.0;
        const double sn = (ln > 1e-12) ? sc / ln : 0.0;
        result.push_back(Point(pts[i][0]+dp[0]*sp, pts[i][1]+dp[1]*sp, pts[i][2]+dp[2]*sp));
        result.push_back(Point(pts[i][0]+dn[0]*sn, pts[i][1]+dn[1]*sn, pts[i][2]+dn[2]*sn));
    }
    return result;
}

/// Newell normal of a face, +z when the face is missing
static Vector fold_face_normal(const Mesh& mesh, size_t fk) {
    const std::optional<std::vector<Point>> pts = mesh.face_points(fk);
    if (!pts) return Vector(0, 0, 1);
    return newell_normal(*pts);
}

/// One miter plane per edge, through the edge midpoint along the averaged neighbor normal
static std::vector<Plane> miter_planes(
    const Mesh& shell,
    const std::map<std::pair<size_t, size_t>, size_t>& efm,
    const std::vector<size_t>& fverts,
    const std::vector<Point>& pts,
    const Vector& fn
) {
    const size_t n = fverts.size();
    std::vector<Plane> planes;
    planes.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        Vector avg_n = fn;
        const auto adj_it = efm.find({fverts[j], fverts[i]});
        if (adj_it != efm.end()) {
            const Vector adj = fold_face_normal(shell, adj_it->second);
            Vector sum(fn[0]+adj[0], fn[1]+adj[1], fn[2]+adj[2]);
            if (sum.magnitude() > 0.1 && sum.normalize_self()) avg_n = sum;
        }
        Vector edge_dir(pts[j][0]-pts[i][0], pts[j][1]-pts[i][1], pts[j][2]-pts[i][2]);
        if (!edge_dir.normalize_self()) return {};
        Vector mn = avg_n.cross(edge_dir);
        if (!mn.normalize_self()) return {};
        const Point mid((pts[i][0]+pts[j][0])/2, (pts[i][1]+pts[j][1])/2, (pts[i][2]+pts[j][2])/2);
        planes.push_back(Plane::from_point_normal(mid, mn));
    }
    return planes;
}

/// Where the corner lines pierce a plane, empty when any misses
static std::vector<Point> miter_contour(const std::vector<Line>& corner_lines, const Plane& plane) {
    std::vector<Point> contour;
    contour.reserve(corner_lines.size());
    for (const Line& line : corner_lines) {
        Point p;
        if (!Intersection::line_plane(line, plane, p, false)) return {};
        contour.push_back(p);
    }
    return contour;
}

std::vector<std::tuple<
    std::vector<Point>, std::vector<Point>,
    std::vector<Point>, std::vector<Point>,
    Vector>>
Mesh::miter_contours(const Mesh& shell, double thickness,
                     double chamfer_bot, double chamfer_top, bool,
                     double chamfer_angle_deg) {
    std::vector<std::tuple<
        std::vector<Point>, std::vector<Point>,
        std::vector<Point>, std::vector<Point>,
        Vector>> result;
    const std::map<std::pair<size_t, size_t>, size_t> efm = shell.edge_face_map();
    for (size_t fk : shell.faces()) {
        const std::optional<std::vector<size_t>> fverts_opt = shell.face_vertices(fk);
        if (!fverts_opt) continue;
        const std::vector<size_t>& fverts = *fverts_opt;
        const size_t n = fverts.size();
        const std::optional<std::vector<Point>> pts = shell.face_points(fk);
        if (!pts || pts->size() != n) continue;
        const Vector fn = newell_normal(*pts);
        const Point cen = ring_centroid(*pts);
        const std::vector<Plane> planes = miter_planes(shell, efm, fverts, *pts, fn);
        if (planes.size() != n) continue;
        std::vector<Line> corner_lines(n);
        bool ok = true;
        for (size_t i = 0; i < n && ok; ++i)
            ok = Intersection::plane_plane(planes[i], planes[(i + 1) % n], corner_lines[i]);
        if (!ok) continue;
        const Point bot_origin(cen[0]+fn[0]*2.0*thickness, cen[1]+fn[1]*2.0*thickness, cen[2]+fn[2]*2.0*thickness);
        const std::vector<Point> top_contour = miter_contour(corner_lines, Plane::from_point_normal(cen, fn));
        const std::vector<Point> bot_contour = miter_contour(corner_lines, Plane::from_point_normal(bot_origin, fn));
        if (top_contour.size() != n || bot_contour.size() != n) continue;
        const std::vector<bool> top_mask = fold_chamfer_mask(top_contour, chamfer_angle_deg);
        const std::vector<bool> bot_mask = fold_chamfer_mask(bot_contour, chamfer_angle_deg);
        const std::vector<Point> top_ch = fold_chamfer(top_contour, chamfer_bot, top_mask);
        const std::vector<Point> bot_ch = fold_chamfer(bot_contour, chamfer_top, bot_mask);
        result.push_back(std::make_tuple(top_ch, bot_ch, top_contour, bot_contour, fn));
    }
    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Boolean Queries
// ═══════════════════════════════════════════════════════════════════════════

bool Mesh::is_valid() const {
    if (vertex.empty() || face.empty()) return false;
    for (const auto& [fkey, vkeys] : face) {
        if (vkeys.size() < 3) return false;
        for (size_t vk : vkeys)
            if (vertex.find(vk) == vertex.end()) return false;
    }
    return true;
}

bool Mesh::is_closed() const {
    std::set<std::pair<size_t,size_t>> hole_edges;
    for (const auto& [fk, rings] : face_holes)
        for (const std::vector<size_t>& ring : rings) {
            const size_t n = ring.size();
            for (size_t i = 0; i < n; ++i) {
                hole_edges.emplace(ring[i], ring[(i + 1) % n]);
                hole_edges.emplace(ring[(i + 1) % n], ring[i]);
            }
        }
    const std::set<std::pair<size_t, size_t>> dfe = directed_face_edges();
    for (const auto& [u, v] : dfe)
        if (dfe.find({v, u}) == dfe.end() && hole_edges.find({v, u}) == hole_edges.end()) return false;
    return !vertex.empty();
}

bool Mesh::is_vertex_on_boundary(size_t vertex_key) const {
    const std::set<std::pair<size_t, size_t>> dfe = directed_face_edges();
    for (const auto& [u, v] : dfe)
        if (dfe.find({v, u}) == dfe.end() && (u == vertex_key || v == vertex_key)) return true;
    return false;
}

bool Mesh::is_edge_on_boundary(size_t u, size_t v) const {
    const std::set<std::pair<size_t, size_t>> dfe = directed_face_edges();
    return !(dfe.count({u, v}) && dfe.count({v, u}));
}

bool Mesh::is_face_on_boundary(size_t face_key) const {
    const std::optional<std::vector<std::pair<size_t, size_t>>> fe = face_edges(face_key);
    if (!fe) return false;
    for (const auto& [u, v] : *fe)
        if (is_edge_on_boundary(u, v)) return true;
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════
// Attributes
// ═══════════════════════════════════════════════════════════════════════════

size_t Mesh::number_of_edges() const {
    const std::set<std::pair<size_t, size_t>> dfe = directed_face_edges();
    size_t count = 0;
    for (const auto& [u, v] : dfe)
        if (u < v || dfe.find({v, u}) == dfe.end()) count++;
    return count;
}

int Mesh::euler() const {
    return static_cast<int>(number_of_vertices()) - static_cast<int>(number_of_edges()) + static_cast<int>(number_of_faces());
}

std::vector<size_t> Mesh::vertices() const {
    std::vector<size_t> result;
    result.reserve(vertex.size());
    for (const auto& [k, _] : vertex)
        result.push_back(k);
    return result;
}

std::vector<size_t> Mesh::faces() const {
    std::vector<size_t> result;
    result.reserve(face.size());
    for (const auto& [k, _] : face)
        result.push_back(k);
    return result;
}

std::vector<std::pair<size_t, size_t>> Mesh::edges() const {
    const std::set<std::pair<size_t, size_t>> dfe = directed_face_edges();
    std::set<std::pair<size_t, size_t>> seen;
    for (const auto& [u, v] : dfe)
        seen.insert(std::minmax(u, v));
    return std::vector<std::pair<size_t, size_t>>(seen.begin(), seen.end());
}

std::pair<std::vector<Point>, std::vector<std::vector<size_t>>> Mesh::to_vertices_and_faces() const {
    const std::map<size_t, size_t> vertex_idx = vertex_index();
    std::vector<Point> vertices(vertex.size());
    for (const auto& [key, vdata] : vertex)
        vertices[vertex_idx.at(key)] = vdata.position();
    std::vector<std::vector<size_t>> faces;
    faces.reserve(face.size());
    for (const auto& [key, face_vertices] : face) {
        std::vector<size_t> remapped;
        remapped.reserve(face_vertices.size());
        for (size_t v : face_vertices)
            remapped.push_back(vertex_idx.at(v));
        faces.push_back(remapped);
    }
    return {vertices, faces};
}

std::map<size_t, size_t> Mesh::vertex_index() const {
    std::map<size_t, size_t> index_map;
    size_t index = 0;
    for (const auto& [key, _] : vertex)
        index_map[key] = index++;
    return index_map;
}

std::vector<std::pair<size_t, size_t>> Mesh::naked_edges(bool boundary) const {
    const std::set<std::pair<size_t, size_t>> dfe = directed_face_edges();
    std::set<std::pair<size_t, size_t>> seen;
    for (const auto& [u, v] : dfe)
        seen.insert(std::minmax(u, v));
    std::vector<std::pair<size_t, size_t>> result;
    for (const auto& [u, v] : seen) {
        const bool naked = !(dfe.count({u, v}) && dfe.count({v, u}));
        if (naked == boundary) result.push_back({u, v});
    }
    return result;
}

std::vector<size_t> Mesh::naked_vertices(bool boundary) const {
    std::vector<size_t> result;
    for (const auto& [vk, _] : vertex)
        if (is_vertex_on_boundary(vk) == boundary) result.push_back(vk);
    return result;
}

std::vector<size_t> Mesh::naked_faces(bool boundary) const {
    std::vector<size_t> result;
    for (const auto& [fk, _] : face)
        if (is_face_on_boundary(fk) == boundary) result.push_back(fk);
    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Vertex and Face Operations
// ═══════════════════════════════════════════════════════════════════════════

size_t Mesh::add_vertex(const Point& position, std::optional<size_t> vkey) {
    ensure_halfedges();
    const size_t vertex_key = vkey.value_or(max_vertex);
    if (vertex_key >= max_vertex) max_vertex = vertex_key + 1;
    vertex[vertex_key] = VertexData(position);
    halfedge[vertex_key] = {};
    pointcolors.push_back(Color::white());
    clear_triangle_bvh();
    return vertex_key;
}

std::optional<size_t> Mesh::add_face(const std::vector<size_t>& vertices, std::optional<size_t> fkey) {
    ensure_halfedges();
    if (vertices.size() < 3) return std::nullopt;
    for (size_t v : vertices)
        if (vertex.find(v) == vertex.end()) return std::nullopt;
    const std::set<size_t> unique_vertices(vertices.begin(), vertices.end());
    if (unique_vertices.size() != vertices.size()) return std::nullopt;
    const size_t face_key = fkey.value_or(max_face);
    if (face_key >= max_face) max_face = face_key + 1;
    face[face_key] = vertices;
    triangulation.erase(face_key);
    facecolors.push_back(Color::white());
    for (size_t i = 0; i < vertices.size(); ++i) {
        const size_t u = vertices[i];
        const size_t v = vertices[(i + 1) % vertices.size()];
        const bool is_new_edge = (halfedge[v].find(u) == halfedge[v].end());
        halfedge[u][v] = face_key;
        if (is_new_edge) {
            halfedge[v][u] = std::nullopt;
            linecolors.push_back(Color::black());
            widths.push_back(1.0);
        }
    }
    clear_triangle_bvh();
    return face_key;
}

void Mesh::remove_vertex(size_t vkey) {
    ensure_halfedges();
    if (vertex.find(vkey) == vertex.end()) return;
    std::vector<size_t> faces_to_remove;
    for (const auto& [fk, verts] : face)
        if (std::find(verts.begin(), verts.end(), vkey) != verts.end()) faces_to_remove.push_back(fk);
    for (size_t fk : faces_to_remove)
        remove_face(fk);
    const auto hit = halfedge.find(vkey);
    if (hit != halfedge.end()) {
        for (const auto& [v, _] : hit->second) {
            const auto vit = halfedge.find(v);
            if (vit != halfedge.end()) vit->second.erase(vkey);
        }
        halfedge.erase(vkey);
    }
    for (auto it = edgedata.begin(); it != edgedata.end(); ) {
        if (it->first.first == vkey || it->first.second == vkey) it = edgedata.erase(it);
        else ++it;
    }
    vertex.erase(vkey);
    if (pointcolors.size() > vertex.size()) pointcolors.resize(vertex.size());
    clear_triangle_bvh();
}

void Mesh::remove_face(size_t fkey) {
    ensure_halfedges();
    const auto it = face.find(fkey);
    if (it == face.end()) return;
    const std::vector<size_t> verts = it->second;
    const size_t n = verts.size();
    for (size_t i = 0; i < n; ++i) {
        const size_t u = verts[i];
        const size_t v = verts[(i + 1) % n];
        const auto uit = halfedge.find(u);
        if (uit == halfedge.end()) continue;
        const auto vit = uit->second.find(v);
        if (vit == uit->second.end()) continue;
        vit->second = std::nullopt;
        const auto vit2 = halfedge.find(v);
        if (vit2 == halfedge.end()) continue;
        const auto uit2 = vit2->second.find(u);
        if (uit2 != vit2->second.end() && !uit2->second.has_value()) {
            uit->second.erase(v);
            vit2->second.erase(u);
        }
    }
    face.erase(fkey);
    triangulation.erase(fkey);
    facedata.erase(fkey);
    face_holes.erase(fkey);
    const size_t n_edges = number_of_edges();
    if (linecolors.size() > n_edges) linecolors.resize(n_edges);
    if (widths.size() > n_edges) widths.resize(n_edges);
    if (facecolors.size() > face.size()) facecolors.resize(face.size());
    clear_triangle_bvh();
}

void Mesh::remove_edge(size_t u, size_t v) {
    ensure_halfedges();
    std::vector<size_t> faces_to_remove;
    const std::optional<size_t> f_uv = halfedge_face({u, v});
    const std::optional<size_t> f_vu = halfedge_face({v, u});
    if (f_uv) faces_to_remove.push_back(*f_uv);
    if (f_vu && f_vu != f_uv) faces_to_remove.push_back(*f_vu);
    for (size_t fk : faces_to_remove)
        remove_face(fk);
    const auto hit = halfedge.find(u);
    if (hit != halfedge.end()) hit->second.erase(v);
    const auto hit2 = halfedge.find(v);
    if (hit2 != halfedge.end()) hit2->second.erase(u);
    edgedata.erase({u, v});
    edgedata.erase({v, u});
    const size_t n_edges = number_of_edges();
    if (linecolors.size() > n_edges) linecolors.resize(n_edges);
    if (widths.size() > n_edges) widths.resize(n_edges);
    clear_triangle_bvh();
}

void Mesh::flip_face(size_t fkey) {
    ensure_halfedges();
    const auto it = face.find(fkey);
    if (it == face.end()) return;
    std::vector<size_t> fv = it->second;
    remove_face(fkey);
    std::reverse(fv.begin(), fv.end());
    add_face(fv, fkey);
}

void Mesh::flip() {
    for (auto& [fkey, verts] : face)
        std::reverse(verts.begin(), verts.end());
    rebuild_halfedges();
}

void Mesh::clear() {
    halfedge.clear();
    vertex.clear();
    face.clear();
    facedata.clear();
    edgedata.clear();
    triangulation.clear();
    face_holes.clear();
    max_vertex = 0;
    max_face = 0;
    pointcolors.clear();
    facecolors.clear();
    linecolors.clear();
    widths.clear();
    objectcolor = Color::white();
    color_mode = ColorMode::OBJECTCOLOR;
    clear_triangle_bvh();
}

Mesh Mesh::unweld() const {
    Mesh m;
    for (const auto& [fkey, vkeys] : face) {
        std::vector<size_t> new_vkeys;
        new_vkeys.reserve(vkeys.size());
        for (size_t vk : vkeys)
            new_vkeys.push_back(m.add_vertex(vertex.at(vk).position()));
        m.add_face(new_vkeys);
    }
    return m;
}

/// Root of x in a union-find forest, halving the path on the way
static size_t weld_find(std::vector<size_t>& parent, size_t x) {
    const size_t bound = parent.size();
    for (size_t step = 0; step < bound && parent[x] != x; ++step) {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    return x;
}

Mesh Mesh::weld(double tolerance) const {
    if (vertex.empty()) return Mesh();
    const std::vector<size_t> vkeys = vertices();
    std::vector<Point> positions;
    positions.reserve(vkeys.size());
    for (size_t vk : vkeys) positions.push_back(vertex.at(vk).position());
    const size_t n = vkeys.size();
    std::vector<size_t> parent(n);
    std::iota(parent.begin(), parent.end(), 0);
    if (tolerance > 0.0) {
        std::vector<OBB> boxes;
        boxes.reserve(n);
        for (const Point& p : positions)
            boxes.push_back(OBB::from_point(p, tolerance));
        const double ws = SpatialBVH::compute_world_size(boxes);
        SpatialBVH bvh = SpatialBVH::from_boxes(boxes, ws);
        const auto [pairs, ignore1, ignore2] = bvh.check_all_collisions(boxes);
        for (const auto& [i, j] : pairs) {
            if (positions[i].distance(positions[j]) > tolerance) continue;
            const size_t ri = weld_find(parent, i), rj = weld_find(parent, j);
            if (ri != rj) parent[ri] = rj;
        }
    }
    std::map<size_t, size_t> root_to_rep;
    for (size_t i = 0; i < n; i++) {
        const size_t root = weld_find(parent, i);
        const auto [it, inserted] = root_to_rep.emplace(root, vkeys[i]);
        if (!inserted && vkeys[i] < it->second) it->second = vkeys[i];
    }
    std::map<size_t, size_t> vkey_to_rep;
    for (size_t i = 0; i < n; i++)
        vkey_to_rep[vkeys[i]] = root_to_rep.at(weld_find(parent, i));
    Mesh m;
    std::set<size_t> added;
    for (size_t i = 0; i < n; i++) {
        const size_t rep = vkey_to_rep.at(vkeys[i]);
        if (added.insert(rep).second)
            m.add_vertex(vertex.at(rep).position(), rep);
    }
    for (const auto& [fk, fvkeys] : face) {
        std::vector<size_t> new_vkeys;
        new_vkeys.reserve(fvkeys.size());
        for (size_t vk : fvkeys) new_vkeys.push_back(vkey_to_rep.at(vk));
        m.add_face(new_vkeys, fk);
    }
    return m;
}

bool Mesh::unify_winding() {
    if (face.size() < 2) return false;
    std::map<std::pair<size_t,size_t>, std::vector<std::tuple<size_t,size_t,size_t>>> edge_faces;
    for (const auto& [fkey, verts] : face) {
        const size_t n = verts.size();
        for (size_t i = 0; i < n; ++i) {
            const size_t u = verts[i];
            const size_t v = verts[(i + 1) % n];
            edge_faces[std::minmax(u, v)].emplace_back(fkey, u, v);
        }
    }
    std::set<size_t> visited;
    std::set<size_t> flipped;
    for (const auto& [seed, _] : face) {
        if (visited.count(seed)) continue;
        visited.insert(seed);
        std::vector<size_t> queue = {seed};
        while (!queue.empty()) {
            const size_t f = queue.back();
            queue.pop_back();
            const bool is_flipped = flipped.count(f) > 0;
            const std::vector<size_t>& verts = face[f];
            const size_t n = verts.size();
            for (size_t i = 0; i < n; ++i) {
                const size_t u_orig = verts[i];
                const size_t v_orig = verts[(i + 1) % n];
                const size_t eff_u = is_flipped ? v_orig : u_orig;
                const size_t eff_v = is_flipped ? u_orig : v_orig;
                for (const auto& [adj_key, adj_u, adj_v] : edge_faces[std::minmax(u_orig, v_orig)]) {
                    if (adj_key == f || visited.count(adj_key)) continue;
                    if (!(adj_u == eff_v && adj_v == eff_u)) flipped.insert(adj_key);
                    visited.insert(adj_key);
                    queue.push_back(adj_key);
                }
            }
        }
    }
    if (flipped.empty()) return false;
    for (size_t fkey : flipped)
        std::reverse(face[fkey].begin(), face[fkey].end());
    rebuild_halfedges();
    orient_outward();
    return true;
}

bool Mesh::orient_outward() {
    ensure_halfedges();
    if (face.empty() || !naked_edges(true).empty()) return false;
    double vol = 0.0;
    for (const auto& [fk, verts] : face) {
        const Point p0 = *vertex_point(verts[0]);
        for (size_t i = 1; i + 1 < verts.size(); ++i) {
            const Point p1 = *vertex_point(verts[i]);
            const Point p2 = *vertex_point(verts[i + 1]);
            vol += p0[0] * (p1[1] * p2[2] - p1[2] * p2[1])
                 + p0[1] * (p1[2] * p2[0] - p1[0] * p2[2])
                 + p0[2] * (p1[0] * p2[1] - p1[1] * p2[0]);
        }
    }
    if (vol >= 0.0) return false;
    for (auto& [fk, verts] : face)
        std::reverse(verts.begin(), verts.end());
    rebuild_halfedges();
    return true;
}

void Mesh::rebuild_halfedges() {
    halfedge = compute_halfedges();
}

void Mesh::ensure_halfedges() {
    if (halfedge.empty() && !face.empty()) rebuild_halfedges();
}

std::set<std::pair<size_t, size_t>> Mesh::directed_face_edges() const {
    std::set<std::pair<size_t, size_t>> s;
    for (const auto& [fkey, verts] : face) {
        const size_t n = verts.size();
        for (size_t i = 0; i < n; ++i)
            s.insert({verts[i], verts[(i + 1) % n]});
    }
    return s;
}

std::map<size_t, std::map<size_t, std::optional<size_t>>> Mesh::compute_halfedges() const {
    std::map<size_t, std::map<size_t, std::optional<size_t>>> he;
    for (const auto& [vkey, _] : vertex)
        he[vkey] = {};
    for (const auto& [fkey, verts] : face) {
        const size_t n = verts.size();
        for (size_t i = 0; i < n; ++i) {
            const size_t u = verts[i];
            const size_t v = verts[(i + 1) % n];
            he[u][v] = fkey;
            if (!he[v].count(u)) he[v][u] = std::nullopt;
        }
    }
    return he;
}

// ═══════════════════════════════════════════════════════════════════════════
// Connectivity Queries
// ═══════════════════════════════════════════════════════════════════════════

/// Sorted neighbors of x over a directed edge set
static std::set<size_t> edge_ends(const std::set<std::pair<size_t, size_t>>& dfe, size_t x) {
    std::set<size_t> keys;
    for (const auto& [a, b] : dfe) {
        if (a == x) keys.insert(b);
        else if (b == x) keys.insert(a);
    }
    return keys;
}

std::optional<std::vector<std::pair<size_t, size_t>>> Mesh::edge_edges(size_t u, size_t v) const {
    const std::set<std::pair<size_t, size_t>> dfe = directed_face_edges();
    if (!dfe.count({u, v}) && !dfe.count({v, u})) return std::nullopt;
    std::vector<std::pair<size_t, size_t>> edges;
    for (size_t w : edge_ends(dfe, u))
        if (w != v) edges.push_back({u, w});
    for (size_t w : edge_ends(dfe, v))
        if (w != u) edges.push_back({v, w});
    return edges;
}

std::optional<std::vector<size_t>> Mesh::edge_faces(size_t u, size_t v) const {
    std::vector<size_t> result;
    for (const auto& [fkey, verts] : face) {
        const size_t n = verts.size();
        for (size_t i = 0; i < n; ++i) {
            const size_t a = verts[i];
            const size_t b = verts[(i + 1) % n];
            if (!((a == u && b == v) || (a == v && b == u))) continue;
            if (std::find(result.begin(), result.end(), fkey) == result.end()) result.push_back(fkey);
        }
    }
    if (result.empty()) return std::nullopt;
    return result;
}

std::map<std::pair<size_t, size_t>, size_t> Mesh::edge_face_map() const {
    std::map<std::pair<size_t, size_t>, size_t> m;
    for (const auto& [fkey, verts] : face) {
        const size_t n = verts.size();
        for (size_t i = 0; i < n; ++i)
            m[{verts[i], verts[(i + 1) % n]}] = fkey;
    }
    return m;
}

std::optional<Line> Mesh::edge_line(size_t u, size_t v) const {
    const std::set<std::pair<size_t, size_t>> dfe = directed_face_edges();
    if (!dfe.count({u, v}) && !dfe.count({v, u})) return std::nullopt;
    const std::optional<Point> pu = vertex_point(u);
    const std::optional<Point> pv = vertex_point(v);
    if (!pu || !pv) return std::nullopt;
    return Line::from_points(*pu, *pv);
}

std::optional<std::vector<std::pair<size_t, size_t>>> Mesh::face_edges(size_t face_key) const {
    const auto it = face.find(face_key);
    if (it == face.end()) return std::nullopt;
    const std::vector<size_t>& verts = it->second;
    const size_t n = verts.size();
    std::vector<std::pair<size_t, size_t>> edges;
    edges.reserve(n);
    for (size_t i = 0; i < n; ++i)
        edges.push_back({verts[i], verts[(i + 1) % n]});
    return edges;
}

std::optional<std::vector<size_t>> Mesh::face_faces(size_t face_key) const {
    const std::optional<std::vector<std::pair<size_t, size_t>>> fe = face_edges(face_key);
    if (!fe) return std::nullopt;
    const std::map<std::pair<size_t, size_t>, size_t> efm = edge_face_map();
    std::vector<size_t> neighbors;
    for (const auto& [u, v] : *fe) {
        const auto it = efm.find({v, u});
        if (it != efm.end()) neighbors.push_back(it->second);
    }
    return neighbors;
}

std::optional<std::vector<Point>> Mesh::face_points(size_t face_key) const {
    const std::optional<std::vector<size_t>> fv = face_vertices(face_key);
    if (!fv) return std::nullopt;
    std::vector<Point> pts;
    pts.reserve(fv->size());
    for (size_t vk : *fv) {
        const std::optional<Point> p = vertex_point(vk);
        if (!p) return std::nullopt;
        pts.push_back(*p);
    }
    return pts;
}

std::optional<Polyline> Mesh::face_polyline(size_t face_key) const {
    const std::optional<std::vector<Point>> pts = face_points(face_key);
    if (!pts) return std::nullopt;
    return Polyline(*pts);
}

std::optional<std::vector<size_t>> Mesh::face_vertices(size_t face_key) const {
    const auto it = face.find(face_key);
    if (it == face.end()) return std::nullopt;
    return it->second;
}

std::optional<std::vector<std::pair<size_t, size_t>>> Mesh::vertex_edges(size_t vertex_key) const {
    const std::optional<std::vector<size_t>> keys = vertex_vertices(vertex_key);
    if (!keys) return std::nullopt;
    std::vector<std::pair<size_t, size_t>> edges;
    edges.reserve(keys->size());
    for (size_t u : *keys)
        edges.push_back({vertex_key, u});
    return edges;
}

std::optional<std::vector<size_t>> Mesh::vertex_faces(size_t vertex_key) const {
    const std::optional<std::vector<size_t>> keys = vertex_vertices(vertex_key);
    if (!keys) return std::nullopt;
    const std::map<std::pair<size_t, size_t>, size_t> efm = edge_face_map();
    std::vector<size_t> faces;
    for (size_t u : *keys) {
        const auto it = efm.find({vertex_key, u});
        if (it != efm.end()) faces.push_back(it->second);
    }
    return faces;
}

std::optional<Point> Mesh::vertex_point(size_t vertex_key) const {
    const auto it = vertex.find(vertex_key);
    if (it == vertex.end()) return std::nullopt;
    return it->second.position();
}

std::optional<std::vector<size_t>> Mesh::vertex_vertices(size_t vertex_key) const {
    if (vertex.find(vertex_key) == vertex.end()) return std::nullopt;
    const std::set<size_t> keys = edge_ends(directed_face_edges(), vertex_key);
    return std::vector<size_t>(keys.begin(), keys.end());
}

std::optional<std::vector<size_t>> Mesh::vertex_neighbors(size_t vertex_key, bool ordered) const {
    const auto it = halfedge.find(vertex_key);
    if (it == halfedge.end()) return std::nullopt;
    std::vector<size_t> nbrs;
    for (const auto& [v, _] : it->second) nbrs.push_back(v);
    if (!ordered || nbrs.size() <= 1) return nbrs;
    size_t start = nbrs[0];
    for (size_t n : nbrs) {
        if (!it->second.at(n).has_value()) { start = n; break; }
    }
    std::optional<size_t> fkey = halfedge_face({start, vertex_key});
    std::vector<size_t> out{ start };
    for (size_t step = 0; step < face.size() && fkey.has_value(); ++step) {
        const auto fit = face.find(*fkey);
        if (fit == face.end()) break;
        const std::vector<size_t>& verts = fit->second;
        const auto pos = std::find(verts.begin(), verts.end(), vertex_key);
        if (pos == verts.end()) break;
        const size_t nbr = verts[(pos - verts.begin() + 1) % verts.size()];
        if (nbr == start) break;
        out.push_back(nbr);
        fkey = halfedge_face({nbr, vertex_key});
    }
    return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// Boundary
// ═══════════════════════════════════════════════════════════════════════════

std::vector<size_t> Mesh::vertices_on_boundary() const {
    std::vector<size_t> out;
    for (const auto& [v, _] : vertex)
        if (is_vertex_on_boundary(v)) out.push_back(v);
    return out;
}

std::vector<std::pair<size_t, size_t>> Mesh::edges_on_boundary() const {
    std::vector<std::pair<size_t, size_t>> out;
    for (const auto& [u, nbrs] : halfedge)
        for (const auto& [v, f] : nbrs)
            if (!f.has_value()) out.emplace_back(u, v);
    return out;
}

std::vector<size_t> Mesh::faces_on_boundary() const {
    std::vector<size_t> out;
    for (const auto& [f, _] : face)
        if (is_face_on_boundary(f)) out.push_back(f);
    return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// Halfedge Navigation
// ═══════════════════════════════════════════════════════════════════════════

std::optional<size_t> Mesh::halfedge_face(std::pair<size_t, size_t> edge) const {
    const auto it = halfedge.find(edge.first);
    if (it == halfedge.end()) return std::nullopt;
    const auto jt = it->second.find(edge.second);
    if (jt == it->second.end()) return std::nullopt;
    return jt->second;
}

std::optional<std::pair<size_t, size_t>> Mesh::halfedge_after(std::pair<size_t, size_t> edge) const {
    const auto [u, v] = edge;
    const std::optional<size_t> f = halfedge_face(edge);
    if (f.has_value()) {
        const auto fit = face.find(*f);
        if (fit == face.end()) return std::nullopt;
        const std::vector<size_t>& verts = fit->second;
        const auto pos = std::find(verts.begin(), verts.end(), v);
        if (pos == verts.end()) return std::nullopt;
        return std::make_pair(v, verts[(pos - verts.begin() + 1) % verts.size()]);
    }
    const auto it = halfedge.find(v);
    if (it == halfedge.end()) return std::nullopt;
    for (const auto& [w, fw] : it->second)
        if (w != u && !fw.has_value()) return std::make_pair(v, w);
    return std::nullopt;
}

std::optional<std::pair<size_t, size_t>> Mesh::halfedge_before(std::pair<size_t, size_t> edge) const {
    const auto [u, v] = edge;
    const std::optional<size_t> f = halfedge_face(edge);
    if (f.has_value()) {
        const auto fit = face.find(*f);
        if (fit == face.end()) return std::nullopt;
        const std::vector<size_t>& verts = fit->second;
        const size_t n = verts.size();
        const auto pos = std::find(verts.begin(), verts.end(), u);
        if (pos == verts.end()) return std::nullopt;
        return std::make_pair(verts[(pos - verts.begin() + n - 1) % n], u);
    }
    const auto it = halfedge.find(u);
    if (it == halfedge.end()) return std::nullopt;
    for (const auto& [w, _] : it->second) {
        if (w == v) continue;
        const auto wit = halfedge.find(w);
        if (wit == halfedge.end()) continue;
        const auto wjt = wit->second.find(u);
        if (wjt != wit->second.end() && !wjt->second.has_value()) return std::make_pair(w, u);
    }
    return std::nullopt;
}

/// Boundary loop from edge: at each vertex continue along the other boundary edge
static std::vector<std::pair<size_t, size_t>> halfedge_loop_boundary(const Mesh& mesh, std::pair<size_t, size_t> edge) {
    std::vector<std::pair<size_t, size_t>> edges{ edge };
    size_t u = edge.first, v = edge.second;
    for (size_t step = 0; step < mesh.vertex.size(); ++step) {
        const std::optional<std::vector<size_t>> nbrs = mesh.vertex_neighbors(v, false);
        if (!nbrs.has_value() || nbrs->size() == 2) break;
        std::optional<size_t> nbr;
        for (size_t temp : *nbrs) {
            if (temp == u) continue;
            if (mesh.is_edge_on_boundary(v, temp)) { nbr = temp; break; }
        }
        if (!nbr.has_value()) break;
        u = v;
        v = *nbr;
        edges.emplace_back(u, v);
        if (v == edges.front().first) break;
    }
    return edges;
}

std::vector<std::pair<size_t, size_t>> Mesh::halfedge_loop(std::pair<size_t, size_t> edge) const {
    if (is_edge_on_boundary(edge.first, edge.second)) return halfedge_loop_boundary(*this, edge);
    std::vector<std::pair<size_t, size_t>> edges{ edge };
    size_t u = edge.first, v = edge.second;
    for (size_t step = 0; step < vertex.size(); ++step) {
        const std::optional<std::vector<size_t>> nbrs = vertex_neighbors(v, true);
        if (!nbrs.has_value() || nbrs->size() != 4) break;
        const auto pos = std::find(nbrs->begin(), nbrs->end(), u);
        if (pos == nbrs->end()) break;
        const size_t i = pos - nbrs->begin();
        u = v;
        v = (*nbrs)[(i + 2) % 4];
        edges.emplace_back(u, v);
        if (v == edges.front().first) break;
    }
    return edges;
}

std::vector<std::pair<size_t, size_t>> Mesh::halfedge_strip(std::pair<size_t, size_t> edge) const {
    size_t u = edge.first, v = edge.second;
    std::vector<std::pair<size_t, size_t>> edges{ edge };
    for (size_t step = 0; step < face.size(); ++step) {
        const std::optional<size_t> fopt = halfedge_face({ u, v });
        if (!fopt.has_value()) break;
        const auto fit = face.find(*fopt);
        if (fit == face.end()) break;
        const std::vector<size_t>& verts = fit->second;
        if (verts.size() != 4) break;
        const auto pos = std::find(verts.begin(), verts.end(), u);
        if (pos == verts.end()) break;
        const size_t i = pos - verts.begin();
        u = verts[(i + 3) % 4];
        v = verts[(i + 2) % 4];
        edges.emplace_back(u, v);
        if (std::make_pair(u, v) == edge) break;
    }
    return edges;
}

// ═══════════════════════════════════════════════════════════════════════════
// Sampling
// ═══════════════════════════════════════════════════════════════════════════

template <typename T>
static std::vector<T> lcg_sample(const std::vector<T>& keys, size_t size, uint32_t seed) {
    if (keys.empty() || size == 0) return {};
    const size_t n = keys.size();
    const size_t take = std::min(size, n);
    if (seed == 0) return std::vector<T>(keys.begin(), keys.begin() + take);
    uint32_t s = seed & 0x7FFFFFFFu;
    if (s == 0) s = 1;
    std::set<size_t> used;
    std::vector<T> out;
    out.reserve(take);
    while (out.size() < take) {
        s = (s * 1103515245u + 12345u) & 0x7FFFFFFFu;
        const size_t i = static_cast<size_t>(s) % n;
        if (used.insert(i).second) out.push_back(keys[i]);
    }
    return out;
}

std::vector<size_t> Mesh::vertex_sample(size_t size, uint32_t seed) const {
    return lcg_sample(vertices(), size, seed);
}

std::vector<std::pair<size_t, size_t>> Mesh::edge_sample(size_t size, uint32_t seed) const {
    return lcg_sample(edges(), size, seed);
}

std::vector<size_t> Mesh::face_sample(size_t size, uint32_t seed) const {
    return lcg_sample(faces(), size, seed);
}

// ═══════════════════════════════════════════════════════════════════════════
// Aliases
// ═══════════════════════════════════════════════════════════════════════════

std::optional<Point> Mesh::face_center(size_t face_key) const { return face_centroid(face_key); }

std::optional<Polyline> Mesh::face_polygon(size_t face_key) const {
    const std::optional<std::vector<Point>> pts_opt = face_points(face_key);
    if (!pts_opt.has_value()) return std::nullopt;
    std::vector<Point> pts = *pts_opt;
    if (!pts.empty() && !(pts.front() == pts.back())) pts.push_back(pts.front());
    return Polyline{ pts };
}

std::vector<Polyline> Mesh::face_outlines() const {
    std::vector<Polyline> outlines;
    outlines.reserve(face.size());
    for (size_t face_key : faces()) {
        const std::optional<Polyline> outline = face_polygon(face_key);
        if (outline && outline->point_count() >= 4) outlines.push_back(*outline);
    }
    return outlines;
}

void Mesh::flip_cycles() { flip(); }

// ═══════════════════════════════════════════════════════════════════════════
// Attribute API
// ═══════════════════════════════════════════════════════════════════════════

void Mesh::update_default_vertex_attributes(const std::vector<std::pair<std::string, double>>& attrs) {
    for (const auto& [k, v] : attrs) default_vertex_attributes[k] = v;
}

void Mesh::update_default_face_attributes(const std::vector<std::pair<std::string, double>>& attrs) {
    for (const auto& [k, v] : attrs) default_face_attributes[k] = v;
}

void Mesh::update_default_edge_attributes(const std::vector<std::pair<std::string, double>>& attrs) {
    for (const auto& [k, v] : attrs) default_edge_attributes[k] = v;
}

std::optional<double> Mesh::vertex_attribute(size_t key, const std::string& name) const {
    const auto it = vertex.find(key);
    if (it == vertex.end()) return std::nullopt;
    const auto ait = it->second.attributes.find(name);
    if (ait != it->second.attributes.end()) return ait->second;
    const auto dit = default_vertex_attributes.find(name);
    if (dit != default_vertex_attributes.end()) return dit->second;
    return std::nullopt;
}

void Mesh::set_vertex_attribute(size_t key, const std::string& name, double value) {
    const auto it = vertex.find(key);
    if (it == vertex.end()) return;
    it->second.attributes[name] = value;
}

std::optional<double> Mesh::face_attribute(size_t fkey, const std::string& name) const {
    if (face.find(fkey) == face.end()) return std::nullopt;
    const auto fit = facedata.find(fkey);
    if (fit != facedata.end()) {
        const auto ait = fit->second.find(name);
        if (ait != fit->second.end()) return ait->second;
    }
    const auto dit = default_face_attributes.find(name);
    if (dit != default_face_attributes.end()) return dit->second;
    return std::nullopt;
}

void Mesh::set_face_attribute(size_t fkey, const std::string& name, double value) {
    if (face.find(fkey) == face.end()) return;
    facedata[fkey][name] = value;
}

std::optional<double> Mesh::edge_attribute(std::pair<size_t, size_t> edge, const std::string& name) const {
    const auto [u, v] = edge;
    const auto uit = halfedge.find(u);
    const auto vit = halfedge.find(v);
    const bool uv = uit != halfedge.end() && uit->second.count(v);
    const bool vu = vit != halfedge.end() && vit->second.count(u);
    if (!uv && !vu) return std::nullopt;
    auto eit = edgedata.find({ u, v });
    if (eit == edgedata.end()) eit = edgedata.find({ v, u });
    if (eit != edgedata.end()) {
        const auto ait = eit->second.find(name);
        if (ait != eit->second.end()) return ait->second;
    }
    const auto dit = default_edge_attributes.find(name);
    if (dit != default_edge_attributes.end()) return dit->second;
    return std::nullopt;
}

void Mesh::set_edge_attribute(std::pair<size_t, size_t> edge, const std::string& name, double value) {
    const auto [u, v] = edge;
    const std::pair<size_t, size_t> key = edgedata.count({ v, u }) ? std::make_pair(v, u) : std::make_pair(u, v);
    edgedata[key][name] = value;
}

std::vector<std::optional<double>> Mesh::vertices_attribute(const std::string& name, const std::vector<size_t>* keys) const {
    const std::vector<size_t> all = vertices();
    if (!keys) keys = &all;
    std::vector<std::optional<double>> out;
    out.reserve(keys->size());
    for (size_t k : *keys) out.push_back(vertex_attribute(k, name));
    return out;
}

void Mesh::set_vertices_attribute(const std::string& name, double value, const std::vector<size_t>* keys) {
    const std::vector<size_t> all = vertices();
    if (!keys) keys = &all;
    for (size_t k : *keys) set_vertex_attribute(k, name, value);
}

std::vector<std::optional<double>> Mesh::faces_attribute(const std::string& name, const std::vector<size_t>* keys) const {
    const std::vector<size_t> all = faces();
    if (!keys) keys = &all;
    std::vector<std::optional<double>> out;
    out.reserve(keys->size());
    for (size_t k : *keys) out.push_back(face_attribute(k, name));
    return out;
}

void Mesh::set_faces_attribute(const std::string& name, double value, const std::vector<size_t>* keys) {
    const std::vector<size_t> all = faces();
    if (!keys) keys = &all;
    for (size_t k : *keys) set_face_attribute(k, name, value);
}

std::vector<std::optional<double>> Mesh::edges_attribute(const std::string& name, const std::vector<std::pair<size_t, size_t>>* keys) const {
    const std::vector<std::pair<size_t, size_t>> all = edges();
    if (!keys) keys = &all;
    std::vector<std::optional<double>> out;
    out.reserve(keys->size());
    for (const std::pair<size_t, size_t>& e : *keys) out.push_back(edge_attribute(e, name));
    return out;
}

void Mesh::set_edges_attribute(const std::string& name, double value, const std::vector<std::pair<size_t, size_t>>* keys) {
    const std::vector<std::pair<size_t, size_t>> all = edges();
    if (!keys) keys = &all;
    for (const std::pair<size_t, size_t>& e : *keys) set_edge_attribute(e, name, value);
}

std::vector<size_t> Mesh::vertices_where(const std::vector<std::pair<std::string, double>>& conditions) const {
    std::vector<size_t> out;
    for (size_t k : vertices()) {
        bool ok = true;
        for (const auto& [n, v] : conditions) {
            const std::optional<double> val = vertex_attribute(k, n);
            if (!val.has_value() || *val != v) { ok = false; break; }
        }
        if (ok) out.push_back(k);
    }
    return out;
}

std::vector<size_t> Mesh::faces_where(const std::vector<std::pair<std::string, double>>& conditions) const {
    std::vector<size_t> out;
    for (size_t k : faces()) {
        bool ok = true;
        for (const auto& [n, v] : conditions) {
            const std::optional<double> val = face_attribute(k, n);
            if (!val.has_value() || *val != v) { ok = false; break; }
        }
        if (ok) out.push_back(k);
    }
    return out;
}

std::vector<std::pair<size_t, size_t>> Mesh::edges_where(const std::vector<std::pair<std::string, double>>& conditions) const {
    std::vector<std::pair<size_t, size_t>> out;
    for (const std::pair<size_t, size_t>& e : edges()) {
        bool ok = true;
        for (const auto& [n, v] : conditions) {
            const std::optional<double> val = edge_attribute(e, n);
            if (!val.has_value() || *val != v) { ok = false; break; }
        }
        if (ok) out.push_back(e);
    }
    return out;
}

std::vector<size_t> Mesh::vertices_where_predicate(const std::function<bool(size_t, const std::map<std::string, double>&)>& pred) const {
    std::vector<size_t> out;
    for (size_t k : vertices()) {
        std::map<std::string, double> attrs = default_vertex_attributes;
        for (const auto& [kk, vv] : vertex.at(k).attributes) attrs[kk] = vv;
        if (pred(k, attrs)) out.push_back(k);
    }
    return out;
}

std::vector<size_t> Mesh::faces_where_predicate(const std::function<bool(size_t, const std::map<std::string, double>&)>& pred) const {
    std::vector<size_t> out;
    for (size_t k : faces()) {
        std::map<std::string, double> attrs = default_face_attributes;
        const auto fit = facedata.find(k);
        if (fit != facedata.end())
            for (const auto& [kk, vv] : fit->second) attrs[kk] = vv;
        if (pred(k, attrs)) out.push_back(k);
    }
    return out;
}

std::vector<std::pair<size_t, size_t>> Mesh::edges_where_predicate(const std::function<bool(std::pair<size_t, size_t>, const std::map<std::string, double>&)>& pred) const {
    std::vector<std::pair<size_t, size_t>> out;
    for (const std::pair<size_t, size_t>& e : edges()) {
        std::map<std::string, double> attrs = default_edge_attributes;
        auto eit = edgedata.find(e);
        if (eit == edgedata.end()) eit = edgedata.find({ e.second, e.first });
        if (eit != edgedata.end())
            for (const auto& [kk, vv] : eit->second) attrs[kk] = vv;
        if (pred(e, attrs)) out.push_back(e);
    }
    return out;
}

std::optional<Vector> Mesh::face_normal_unitized(size_t face_key, bool unitized) const {
    const std::optional<std::vector<size_t>> vertices_opt = face_vertices(face_key);
    if (!vertices_opt.has_value() || vertices_opt->size() < 3) return std::nullopt;
    const std::vector<size_t>& vertices = *vertices_opt;
    const std::optional<Point> p0 = vertex_point(vertices[0]);
    const std::optional<Point> p1 = vertex_point(vertices[1]);
    const std::optional<Point> p2 = vertex_point(vertices[2]);
    if (!p0 || !p1 || !p2) return std::nullopt;
    const Vector u((*p1)[0] - (*p0)[0], (*p1)[1] - (*p0)[1], (*p1)[2] - (*p0)[2]);
    const Vector v((*p2)[0] - (*p0)[0], (*p2)[1] - (*p0)[1], (*p2)[2] - (*p0)[2]);
    const Vector normal = u.cross(v);
    if (!unitized) return normal;
    const double len = normal.magnitude();
    if (len > Tolerance::ZERO_TOLERANCE) return Vector(normal[0] / len, normal[1] / len, normal[2] / len);
    return std::nullopt;
}

// ═══════════════════════════════════════════════════════════════════════════
// Geometric Properties
// ═══════════════════════════════════════════════════════════════════════════

double Mesh::area() const {
    double total = 0.0;
    for (const auto& [fk, _] : face) {
        const std::optional<double> a = face_area(fk);
        if (a) total += *a;
    }
    return total;
}

Point Mesh::centroid() const {
    double x = 0, y = 0, z = 0;
    for (const auto& [vk, v] : vertex) {
        x += v.x;
        y += v.y;
        z += v.z;
    }
    const double n = vertex.empty() ? 1.0 : static_cast<double>(vertex.size());
    return Point(x / n, y / n, z / n);
}

std::optional<double> Mesh::dihedral_angle(size_t u, size_t v) const {
    const std::optional<std::vector<size_t>> ef = edge_faces(u, v);
    if (!ef || ef->size() < 2) return std::nullopt;
    const std::optional<Vector> n0 = face_normal((*ef)[0]);
    const std::optional<Vector> n1 = face_normal((*ef)[1]);
    if (!n0.has_value() || !n1.has_value()) return std::nullopt;
    const double dot = std::clamp(n0->dot(*n1), -1.0, 1.0);
    return (Tolerance::PI - std::acos(dot)) * 180.0 / Tolerance::PI;
}

/// Unit direction from the edge midpoint to the face centroid, in the plane perpendicular to the edge
static std::optional<Vector> dihedral_arm(const Point& centroid, const Point& mid, const Vector& edge) {
    Vector d(centroid[0]-mid[0], centroid[1]-mid[1], centroid[2]-mid[2]);
    const double dot = d.dot(edge);
    d = Vector(d[0]-dot*edge[0], d[1]-dot*edge[1], d[2]-dot*edge[2]);
    const double len = d.magnitude();
    if (len < 1e-10) return std::nullopt;
    return Vector(d[0]/len, d[1]/len, d[2]/len);
}

std::tuple<std::map<std::pair<size_t,size_t>,double>, std::vector<Polyline>, std::vector<Point>>
Mesh::dihedral_angles(double scale, bool with_arcs, bool with_points) const {
    std::map<std::pair<size_t,size_t>,double> angles;
    std::vector<Polyline> arcs;
    std::vector<Point> points;
    const int arc_n = 12;
    const Color label_color = Color::yellow();
    for (const auto& [u, v] : edges()) {
        const std::optional<double> da = dihedral_angle(u, v);
        if (!da) continue;
        angles[{u, v}] = *da;
        const Point ep0 = *vertex_point(u);
        const Point ep1 = *vertex_point(v);
        const Point mid((ep0[0]+ep1[0])*0.5, (ep0[1]+ep1[1])*0.5, (ep0[2]+ep1[2])*0.5);
        if (scale == 0.0) {
            if (!with_points) continue;
            Point pt(mid[0], mid[1], mid[2], std::to_string(*da));
            pt.pointcolor = label_color;
            points.push_back(pt);
            continue;
        }
        const std::vector<size_t> ef = *edge_faces(u, v);
        Vector edge(ep1[0]-ep0[0], ep1[1]-ep0[1], ep1[2]-ep0[2]);
        if (edge.magnitude() < 1e-10 || !edge.normalize_self()) continue;
        const std::optional<Vector> d0 = dihedral_arm(*face_centroid(ef[0]), mid, edge);
        const std::optional<Vector> d1 = dihedral_arm(*face_centroid(ef[1]), mid, edge);
        if (!d0 || !d1) continue;
        const double theta = std::acos(std::clamp(d0->dot(*d1), -1.0, 1.0));
        if (std::abs(std::sin(theta)) < 1e-10) continue;
        std::vector<Point> arc_pts;
        arc_pts.reserve(arc_n + 1);
        for (int j = 0; j <= arc_n; j++) {
            const double t = static_cast<double>(j) / arc_n;
            const double w1 = std::sin((1.0-t)*theta) / std::sin(theta);
            const double w2 = std::sin(t*theta) / std::sin(theta);
            arc_pts.push_back(Point(
                mid[0]+(w1*(*d0)[0]+w2*(*d1)[0])*scale,
                mid[1]+(w1*(*d0)[1]+w2*(*d1)[1])*scale,
                mid[2]+(w1*(*d0)[2]+w2*(*d1)[2])*scale));
        }
        if (with_arcs) {
            Polyline arc(arc_pts);
            arc.name = "dihedral_e"+std::to_string(u)+"_"+std::to_string(v)+"="+std::to_string(*da);
            arc.linecolor = label_color;
            arcs.push_back(arc);
        }
        if (with_points) {
            Point pt(arc_pts[arc_n/2][0], arc_pts[arc_n/2][1], arc_pts[arc_n/2][2], std::to_string(*da));
            pt.pointcolor = label_color;
            points.push_back(pt);
        }
    }
    return {angles, arcs, points};
}

std::optional<double> Mesh::face_area(size_t face_key) const {
    const std::optional<std::vector<size_t>> vertices_opt = face_vertices(face_key);
    if (!vertices_opt.has_value() || vertices_opt->size() < 3) return 0.0;
    const std::vector<size_t>& vertices = *vertices_opt;
    const std::optional<Point> p0 = vertex_point(vertices[0]);
    if (!p0) return std::nullopt;
    double area = 0.0;
    for (size_t i = 1; i + 1 < vertices.size(); ++i) {
        const std::optional<Point> p1 = vertex_point(vertices[i]);
        const std::optional<Point> p2 = vertex_point(vertices[i + 1]);
        if (!p1 || !p2) return std::nullopt;
        const Vector u((*p1)[0] - (*p0)[0], (*p1)[1] - (*p0)[1], (*p1)[2] - (*p0)[2]);
        const Vector v((*p2)[0] - (*p0)[0], (*p2)[1] - (*p0)[1], (*p2)[2] - (*p0)[2]);
        area += u.cross(v).magnitude() * 0.5;
    }
    return area;
}

std::optional<Point> Mesh::face_centroid(size_t face_key) const {
    const std::optional<std::vector<size_t>> verts = face_vertices(face_key);
    if (!verts || verts->empty()) return std::nullopt;
    double x = 0, y = 0, z = 0;
    for (size_t vk : *verts) {
        const std::optional<Point> p = vertex_point(vk);
        if (!p) return std::nullopt;
        x += (*p)[0]; y += (*p)[1]; z += (*p)[2];
    }
    const double n = static_cast<double>(verts->size());
    return Point(x / n, y / n, z / n);
}

std::optional<Vector> Mesh::face_normal(size_t face_key) const {
    return face_normal_unitized(face_key, true);
}

std::map<size_t, Vector> Mesh::face_normals() const {
    std::map<size_t, Vector> normals;
    for (const auto& [face_key, _] : face) {
        const std::optional<Vector> normal = face_normal(face_key);
        if (normal) normals[face_key] = *normal;
    }
    return normals;
}

std::optional<double> Mesh::vertex_angle_in_face(size_t vertex_key, size_t face_key) const {
    const std::optional<std::vector<size_t>> vertices_opt = face_vertices(face_key);
    if (!vertices_opt) return std::nullopt;
    const std::vector<size_t>& vertices = *vertices_opt;
    const auto it = std::find(vertices.begin(), vertices.end(), vertex_key);
    if (it == vertices.end()) return std::nullopt;
    const size_t vertex_index = std::distance(vertices.begin(), it);
    const size_t n = vertices.size();
    const std::optional<Point> center = vertex_point(vertex_key);
    const std::optional<Point> prev_pos = vertex_point(vertices[(vertex_index + n - 1) % n]);
    const std::optional<Point> next_pos = vertex_point(vertices[(vertex_index + 1) % n]);
    if (!center || !prev_pos || !next_pos) return std::nullopt;
    const Vector u((*prev_pos)[0] - (*center)[0], (*prev_pos)[1] - (*center)[1], (*prev_pos)[2] - (*center)[2]);
    const Vector v((*next_pos)[0] - (*center)[0], (*next_pos)[1] - (*center)[1], (*next_pos)[2] - (*center)[2]);
    const double u_len = u.magnitude();
    const double v_len = v.magnitude();
    if (u_len < Tolerance::ZERO_TOLERANCE || v_len < Tolerance::ZERO_TOLERANCE) return 0.0;
    return std::acos(std::clamp(u.dot(v) / (u_len * v_len), -1.0, 1.0));
}

std::optional<Vector> Mesh::vertex_normal(size_t vertex_key) const {
    return vertex_normal_weighted(vertex_key, NormalWeighting::Area);
}

std::optional<Vector> Mesh::vertex_normal_weighted(size_t vertex_key, NormalWeighting weighting) const {
    const std::optional<std::vector<size_t>> faces_opt = vertex_faces(vertex_key);
    if (!faces_opt || faces_opt->empty()) return std::nullopt;
    Vector normal_acc(0.0, 0.0, 0.0);
    for (size_t face_key : *faces_opt) {
        const std::optional<Vector> fn = face_normal(face_key);
        if (!fn) continue;
        double weight = 1.0;
        if (weighting == NormalWeighting::Area) weight = face_area(face_key).value_or(1.0);
        else if (weighting == NormalWeighting::Angle) weight = vertex_angle_in_face(vertex_key, face_key).value_or(1.0);
        normal_acc[0] = normal_acc[0] + (*fn)[0] * weight;
        normal_acc[1] = normal_acc[1] + (*fn)[1] * weight;
        normal_acc[2] = normal_acc[2] + (*fn)[2] * weight;
    }
    const double len = normal_acc.magnitude();
    if (len > Tolerance::ZERO_TOLERANCE) return Vector(normal_acc[0] / len, normal_acc[1] / len, normal_acc[2] / len);
    return std::nullopt;
}

std::map<size_t, Vector> Mesh::vertex_normals() const {
    return vertex_normals_weighted(NormalWeighting::Area);
}

/// Corner weight of vertex i in a face: its interior angle
static double corner_angle(const std::vector<Point>& pts, size_t i) {
    const size_t n = pts.size();
    const size_t prev = (i + n - 1) % n, next = (i + 1) % n;
    const Vector a(pts[prev][0]-pts[i][0], pts[prev][1]-pts[i][1], pts[prev][2]-pts[i][2]);
    const Vector b(pts[next][0]-pts[i][0], pts[next][1]-pts[i][1], pts[next][2]-pts[i][2]);
    const double a_len = a.magnitude();
    const double b_len = b.magnitude();
    if (a_len < Tolerance::ZERO_TOLERANCE || b_len < Tolerance::ZERO_TOLERANCE) return 0.0;
    return std::acos(std::clamp(a.dot(b) / (a_len * b_len), -1.0, 1.0));
}

std::map<size_t, Vector> Mesh::vertex_normals_weighted(NormalWeighting weighting) const {
    std::map<size_t, Vector> acc;
    for (const auto& [fk, vkeys] : face) {
        if (vkeys.size() < 3) continue;
        const std::optional<std::vector<Point>> pts = face_points(fk);
        if (!pts) continue;
        const std::optional<Vector> normal = face_normal(fk);
        if (!normal) continue;
        double area = 0.0;
        if (weighting == NormalWeighting::Area) area = *face_area(fk);
        for (size_t i = 0; i < vkeys.size(); ++i) {
            double weight = 1.0;
            if (weighting == NormalWeighting::Area) weight = area;
            else if (weighting == NormalWeighting::Angle) weight = corner_angle(*pts, i);
            Vector& v = acc[vkeys[i]];
            v[0] = v[0] + (*normal)[0] * weight;
            v[1] = v[1] + (*normal)[1] * weight;
            v[2] = v[2] + (*normal)[2] * weight;
        }
    }
    std::map<size_t, Vector> normals;
    for (const auto& [vk, v] : acc) {
        const double len = v.magnitude();
        if (len > Tolerance::ZERO_TOLERANCE) normals[vk] = Vector(v[0] / len, v[1] / len, v[2] / len);
    }
    return normals;
}

double Mesh::volume() const {
    double total = 0.0;
    for (const auto& [fk, vkeys] : face) {
        if (vkeys.size() < 3) continue;
        const std::optional<Point> p0 = vertex_point(vkeys[0]);
        if (!p0) continue;
        for (size_t i = 1; i + 1 < vkeys.size(); ++i) {
            const std::optional<Point> p1 = vertex_point(vkeys[i]);
            const std::optional<Point> p2 = vertex_point(vkeys[i + 1]);
            if (!p1 || !p2) continue;
            total += (*p0)[0] * ((*p1)[1] * (*p2)[2] - (*p1)[2] * (*p2)[1])
                   + (*p0)[1] * ((*p1)[2] * (*p2)[0] - (*p1)[0] * (*p2)[2])
                   + (*p0)[2] * ((*p1)[0] * (*p2)[1] - (*p1)[1] * (*p2)[0]);
        }
    }
    return std::abs(total) / 6.0;
}

// ═══════════════════════════════════════════════════════════════════════════
// Triangle BVH
// ═══════════════════════════════════════════════════════════════════════════

struct TriangleTask { uint32_t i0, i1, i2; size_t face_idx; size_t sub_idx; };

/// Every triangle of the mesh: the stored triangulation of an n-gon, a fan from vertex 0 otherwise
static std::vector<TriangleTask> triangle_tasks(const Mesh& mesh, const std::vector<std::vector<size_t>>& faces) {
    const std::map<size_t, size_t> vkey_to_idx = mesh.vertex_index();
    const std::vector<size_t> face_keys = mesh.faces();
    const std::map<size_t, std::vector<std::array<size_t, 3>>>& triangulation = mesh.get_triangulation();
    std::vector<TriangleTask> tasks;
    for (size_t fi = 0; fi < faces.size(); ++fi) {
        const std::vector<size_t>& fv = faces[fi];
        if (fv.size() < 3) continue;
        const auto it = fv.size() >= 5 ? triangulation.find(face_keys[fi]) : triangulation.end();
        if (it != triangulation.end()) {
            for (size_t j = 0; j < it->second.size(); ++j) {
                const std::array<size_t, 3>& t = it->second[j];
                tasks.push_back({static_cast<uint32_t>(vkey_to_idx.at(t[0])), static_cast<uint32_t>(vkey_to_idx.at(t[1])), static_cast<uint32_t>(vkey_to_idx.at(t[2])), fi, j});
            }
            continue;
        }
        for (size_t j = 1; j + 1 < fv.size(); ++j)
            tasks.push_back({static_cast<uint32_t>(fv[0]), static_cast<uint32_t>(fv[j]), static_cast<uint32_t>(fv[j+1]), fi, j});
    }
    return tasks;
}

/// AABB of a triangle, padded by a thousandth
static AABB triangle_aabb(const Point& p0, const Point& p1, const Point& p2) {
    const double min_x = std::min({p0[0], p1[0], p2[0]}) - 0.001;
    const double min_y = std::min({p0[1], p1[1], p2[1]}) - 0.001;
    const double min_z = std::min({p0[2], p1[2], p2[2]}) - 0.001;
    const double max_x = std::max({p0[0], p1[0], p2[0]}) + 0.001;
    const double max_y = std::max({p0[1], p1[1], p2[1]}) + 0.001;
    const double max_z = std::max({p0[2], p1[2], p2[2]}) + 0.001;
    return AABB{(min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5,
                (max_x - min_x) * 0.5, (max_y - min_y) * 0.5, (max_z - min_z) * 0.5};
}

/// World size for the BVH Morton grid: 2.2 times the largest absolute extent, at least 10
static double triangle_world_size(const std::vector<AABB>& aabbs) {
    double extent = 0.0;
    for (const AABB& bb : aabbs) {
        extent = std::max(extent, std::fabs(bb.cx - bb.hx));
        extent = std::max(extent, std::fabs(bb.cx + bb.hx));
        extent = std::max(extent, std::fabs(bb.cy - bb.hy));
        extent = std::max(extent, std::fabs(bb.cy + bb.hy));
        extent = std::max(extent, std::fabs(bb.cz - bb.hz));
        extent = std::max(extent, std::fabs(bb.cz + bb.hz));
    }
    return std::max(2.2 * extent, 10.0);
}

void Mesh::build_triangle_bvh(bool force) const {
    if (triangle_bvh_built && !force) return;
    clear_triangle_bvh();
    const auto [vertices, faces] = to_vertices_and_faces();
    vertices_cache = vertices;
    const std::vector<TriangleTask> tasks = triangle_tasks(*this, faces);
    triangle_aabbs_cache.resize(tasks.size());
    triangle_indices_cache.resize(tasks.size());
    triangle_face_subidx_cache.resize(tasks.size());
    parallel_for(tasks.size(), [&](size_t k) {
        const TriangleTask& t = tasks[k];
        triangle_aabbs_cache[k] = triangle_aabb(vertices_cache[t.i0], vertices_cache[t.i1], vertices_cache[t.i2]);
        triangle_indices_cache[k] = TriangleIndex{t.i0, t.i1, t.i2};
        triangle_face_subidx_cache[k] = {t.face_idx, t.sub_idx};
    });
    triangle_bvh = std::make_shared<SpatialBVH>();
    triangle_bvh->build_from_aabbs(triangle_aabbs_cache.data(), triangle_aabbs_cache.size(), triangle_world_size(triangle_aabbs_cache));
    triangle_bvh_built = true;
}

bool Mesh::triangle_bvh_ray_cast(const Point& origin, const Vector& direction, std::vector<int>& candidate_ids, bool find_all) const {
    build_triangle_bvh(false);
    if (!triangle_bvh) return false;
    return triangle_bvh->ray_cast(origin, direction, candidate_ids, find_all);
}

bool Mesh::get_triangle_by_id(int tri_id, size_t& face_idx, size_t& sub_idx, Point& v0, Point& v1, Point& v2) const {
    if (tri_id < 0) return false;
    const size_t id = static_cast<size_t>(tri_id);
    if (id >= triangle_indices_cache.size() || id >= triangle_face_subidx_cache.size()) return false;
    const TriangleIndex& tri = triangle_indices_cache[id];
    face_idx = triangle_face_subidx_cache[id].first;
    sub_idx = triangle_face_subidx_cache[id].second;
    if (tri.i0 >= vertices_cache.size() || tri.i1 >= vertices_cache.size() || tri.i2 >= vertices_cache.size()) return false;
    v0 = vertices_cache[tri.i0];
    v1 = vertices_cache[tri.i1];
    v2 = vertices_cache[tri.i2];
    return true;
}

void Mesh::clear_triangle_bvh() const {
    triangle_bvh_built = false;
    triangle_bvh.reset();
    triangle_aabb_tree.reset();
    triangle_aabbs_cache.clear();
    triangle_indices_cache.clear();
    triangle_face_subidx_cache.clear();
    vertices_cache.clear();
}

void Mesh::build_triangle_aabb_tree(bool force) const {
    build_triangle_bvh(false);
    if (triangle_aabb_tree && !force) return;
    triangle_aabb_tree = std::make_shared<SpatialAABBTree>();
    triangle_aabb_tree->build(triangle_aabbs_cache.data(), triangle_aabbs_cache.size());
}

// ═══════════════════════════════════════════════════════════════════════════
// Transformation
// ═══════════════════════════════════════════════════════════════════════════

bool Mesh::transform(const Xform& xf) {
    for (auto& [idx, vdata] : vertex) {
        Point pt(vdata.x, vdata.y, vdata.z);
        pt.transform(xf);
        vdata.set_position(pt);
    }
    clear_triangle_bvh();
    return true;
}

Mesh Mesh::transformed(const Xform& xf) const {
    Mesh result = *this;
    result.transform(xf);
    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════

/// Colors as a flat [r, g, b, a, ...] array
static nlohmann::ordered_json colors_to_json(const std::vector<Color>& colors) {
    nlohmann::ordered_json arr = nlohmann::ordered_json::array();
    for (const Color& c : colors) {
        arr.push_back(c.r);
        arr.push_back(c.g);
        arr.push_back(c.b);
        arr.push_back(c.a);
    }
    return arr;
}

/// Colors from a flat [r, g, b, a, ...] array
static std::vector<Color> colors_from_json(const nlohmann::json& arr) {
    std::vector<Color> colors;
    if (!arr.is_array()) return colors;
    for (size_t i = 0; i + 3 < arr.size(); i += 4)
        colors.push_back(Color(arr[i].get<float>(), arr[i+1].get<float>(), arr[i+2].get<float>(), arr[i+3].get<float>()));
    return colors;
}

nlohmann::ordered_json Mesh::jsondump() const {
    nlohmann::ordered_json data;
    data["color_mode"] = color_mode_to_string(color_mode);
    data["default_edge_attributes"] = default_edge_attributes;
    data["default_face_attributes"] = default_face_attributes;
    data["default_vertex_attributes"] = default_vertex_attributes;
    nlohmann::ordered_json edgedata_json = nlohmann::ordered_json::object();
    for (const auto& [edge, attrs] : edgedata)
        edgedata_json[std::to_string(edge.first) + "," + std::to_string(edge.second)] = attrs;
    data["edgedata"] = edgedata_json;
    nlohmann::ordered_json face_json = nlohmann::ordered_json::object();
    for (const auto& [key, vertices] : face)
        face_json[std::to_string(key)] = vertices;
    data["face"] = face_json;
    nlohmann::ordered_json face_holes_json = nlohmann::ordered_json::object();
    for (const auto& [fkey, rings] : face_holes)
        face_holes_json[std::to_string(fkey)] = rings;
    data["face_holes"] = face_holes_json;
    data["facecolors"] = colors_to_json(facecolors);
    nlohmann::ordered_json facedata_json = nlohmann::ordered_json::object();
    for (const auto& [key, attrs] : facedata)
        facedata_json[std::to_string(key)] = attrs;
    data["facedata"] = facedata_json;
    data["guid"] = guid();
    const std::map<size_t, std::map<size_t, std::optional<size_t>>> he = halfedge.empty() && !face.empty() ? compute_halfedges() : halfedge;
    nlohmann::ordered_json halfedge_json = nlohmann::ordered_json::object();
    for (const auto& [u, neighbors] : he) {
        nlohmann::ordered_json neighbor_json = nlohmann::ordered_json::object();
        for (const auto& [v, face_opt] : neighbors)
            neighbor_json[std::to_string(v)] = face_opt.has_value() ? nlohmann::json(face_opt.value()) : nlohmann::json(nullptr);
        halfedge_json[std::to_string(u)] = neighbor_json;
    }
    data["halfedge"] = halfedge_json;
    data["linecolors"] = colors_to_json(linecolors);
    data["max_face"] = max_face;
    data["max_vertex"] = max_vertex;
    data["name"] = name;
    data["objectcolor"] = objectcolor.jsondump();
    data["pointcolors"] = colors_to_json(pointcolors);
    nlohmann::ordered_json triangulation_json = nlohmann::ordered_json::object();
    for (const auto& [fkey, tris] : triangulation) {
        nlohmann::json tri_arr = nlohmann::json::array();
        for (const std::array<size_t, 3>& t : tris)
            tri_arr.push_back({t[0], t[1], t[2]});
        triangulation_json[std::to_string(fkey)] = tri_arr;
    }
    data["triangulation"] = triangulation_json;
    data["type"] = "Mesh";
    nlohmann::ordered_json vertex_json = nlohmann::ordered_json::object();
    for (const auto& [key, vdata] : vertex) {
        nlohmann::ordered_json v;
        v["attributes"] = vdata.attributes;
        v["x"] = vdata.x;
        v["y"] = vdata.y;
        v["z"] = vdata.z;
        vertex_json[std::to_string(key)] = v;
    }
    data["vertex"] = vertex_json;
    data["widths"] = widths;
    return data;
}

Mesh Mesh::jsonload(const nlohmann::json& data) {
    Mesh mesh;
    if (data.contains("guid")) mesh.guid() = data["guid"];
    if (data.contains("name")) mesh.name = data["name"];
    if (data.contains("halfedge")) {
        for (const auto& [u_str, neighbors] : data["halfedge"].items()) {
            const size_t u = std::stoull(u_str);
            mesh.halfedge[u] = {};
            for (const auto& [v_str, face_val] : neighbors.items()) {
                const size_t v = std::stoull(v_str);
                if (face_val.is_null()) mesh.halfedge[u][v] = std::nullopt;
                else mesh.halfedge[u][v] = face_val.get<size_t>();
            }
        }
    }
    if (data.contains("vertex")) {
        for (const auto& [key_str, vdata] : data["vertex"].items()) {
            const size_t key = std::stoull(key_str);
            VertexData vertex_data;
            vertex_data.x = vdata["x"];
            vertex_data.y = vdata["y"];
            vertex_data.z = vdata["z"];
            if (vdata.contains("attributes")) vertex_data.attributes = vdata["attributes"].get<std::map<std::string, double>>();
            mesh.vertex[key] = vertex_data;
            if (!data.contains("halfedge")) mesh.halfedge[key] = {};
            if (key >= mesh.max_vertex) mesh.max_vertex = key + 1;
        }
    }
    if (data.contains("face")) {
        for (const auto& [key_str, vertices] : data["face"].items()) {
            const size_t key = std::stoull(key_str);
            mesh.face[key] = vertices.get<std::vector<size_t>>();
            if (key >= mesh.max_face) mesh.max_face = key + 1;
        }
    }
    if (data.contains("face_holes"))
        for (const auto& [fk_str, rings] : data["face_holes"].items())
            mesh.face_holes[std::stoull(fk_str)] = rings.get<std::vector<std::vector<size_t>>>();
    if (data.contains("facedata"))
        for (const auto& [key_str, attrs] : data["facedata"].items())
            mesh.facedata[std::stoull(key_str)] = attrs.get<std::map<std::string, double>>();
    if (data.contains("edgedata")) {
        for (const auto& [edge_str, attrs] : data["edgedata"].items()) {
            const size_t comma_pos = edge_str.find(',');
            const size_t u = std::stoull(edge_str.substr(0, comma_pos));
            const size_t v = std::stoull(edge_str.substr(comma_pos + 1));
            mesh.edgedata[{u, v}] = attrs.get<std::map<std::string, double>>();
        }
    }
    if (data.contains("default_vertex_attributes")) mesh.default_vertex_attributes = data["default_vertex_attributes"];
    if (data.contains("default_face_attributes")) mesh.default_face_attributes = data["default_face_attributes"];
    if (data.contains("default_edge_attributes")) mesh.default_edge_attributes = data["default_edge_attributes"];
    if (data.contains("max_vertex")) mesh.max_vertex = data["max_vertex"];
    if (data.contains("max_face")) mesh.max_face = data["max_face"];
    if (data.contains("pointcolors")) mesh.pointcolors = colors_from_json(data["pointcolors"]);
    if (data.contains("facecolors")) mesh.facecolors = colors_from_json(data["facecolors"]);
    if (data.contains("linecolors")) mesh.linecolors = colors_from_json(data["linecolors"]);
    if (data.contains("widths") && data["widths"].is_array()) mesh.widths = data["widths"].get<std::vector<double>>();
    if (data.contains("objectcolor")) mesh.objectcolor = Color::jsonload(data["objectcolor"]);
    if (data.contains("color_mode")) mesh.color_mode = color_mode_from_string(data["color_mode"].get<std::string>());
    if (data.contains("triangulation")) {
        for (const auto& [fk_str, tris_val] : data["triangulation"].items()) {
            std::vector<std::array<size_t,3>> tris;
            for (const auto& t : tris_val)
                tris.push_back({t[0].get<size_t>(), t[1].get<size_t>(), t[2].get<size_t>()});
            mesh.triangulation[std::stoull(fk_str)] = tris;
        }
    }
    return mesh;
}

std::string Mesh::file_json_dumps() const {
    return jsondump().dump();
}

Mesh Mesh::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Mesh::file_json_dump(const std::string& filename) const {
    std::ofstream f(filename);
    f << jsondump().dump(2);
}

Mesh Mesh::file_json_load(const std::string& filename) {
    std::ifstream f(filename);
    nlohmann::json j;
    f >> j;
    return jsonload(j);
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════

/// Append colors as flat r, g, b, a floats
static void colors_to_rgba(const std::vector<Color>& colors, google::protobuf::RepeatedField<float>* rgba) {
    for (const Color& c : colors) {
        rgba->Add(c.r);
        rgba->Add(c.g);
        rgba->Add(c.b);
        rgba->Add(c.a);
    }
}

/// Colors from flat r, g, b, a floats
static std::vector<Color> colors_from_rgba(const google::protobuf::RepeatedField<float>& rgba) {
    std::vector<Color> colors;
    for (int i = 0; i + 3 < rgba.size(); i += 4)
        colors.emplace_back(rgba[i], rgba[i+1], rgba[i+2], rgba[i+3]);
    return colors;
}

std::string Mesh::pb_dumps() const {
    session_proto::Mesh proto;
    if (has_guid()) proto.set_guid(guid());
    proto.set_name(name);
    for (const auto& [vkey, vdata] : vertex) {
        session_proto::VertexData& vertex_proto = (*proto.mutable_vertices())[vkey];
        vertex_proto.set_x(vdata.x);
        vertex_proto.set_y(vdata.y);
        vertex_proto.set_z(vdata.z);
        for (const auto& [k, v] : vdata.attributes)
            (*vertex_proto.mutable_attributes())[k] = v;
    }
    for (const auto& [fkey, fverts] : face) {
        session_proto::FaceData& face_proto = (*proto.mutable_faces())[fkey];
        for (size_t v : fverts)
            face_proto.add_vertices(v);
        const auto it = facedata.find(fkey);
        if (it != facedata.end())
            for (const auto& [k, v] : it->second)
                (*face_proto.mutable_attributes())[k] = v;
        const auto hit = face_holes.find(fkey);
        if (hit != face_holes.end()) {
            for (const std::vector<size_t>& ring : hit->second) {
                session_proto::HoleRing* hole_proto = face_proto.add_holes();
                for (size_t v : ring) hole_proto->add_vertices(v);
            }
        }
    }
    for (const auto& [fkey, tris] : triangulation) {
        session_proto::TriList& tri_list = (*proto.mutable_triangulation())[fkey];
        for (const std::array<size_t, 3>& t : tris) {
            tri_list.add_vertices(t[0]);
            tri_list.add_vertices(t[1]);
            tri_list.add_vertices(t[2]);
        }
    }
    for (const auto& [edge, attrs] : edgedata) {
        session_proto::EdgeData* edge_proto = proto.add_edge_data();
        edge_proto->set_vertex1(edge.first);
        edge_proto->set_vertex2(edge.second);
        for (const auto& [k, v] : attrs)
            (*edge_proto->mutable_attributes())[k] = v;
    }
    for (const auto& [k, v] : default_vertex_attributes)
        (*proto.mutable_default_vertex_attributes())[k] = v;
    for (const auto& [k, v] : default_face_attributes)
        (*proto.mutable_default_face_attributes())[k] = v;
    for (const auto& [k, v] : default_edge_attributes)
        (*proto.mutable_default_edge_attributes())[k] = v;
    colors_to_rgba(pointcolors, proto.mutable_pointcolors_rgba());
    colors_to_rgba(facecolors, proto.mutable_facecolors_rgba());
    colors_to_rgba(linecolors, proto.mutable_linecolors_rgba());
    for (double w : widths)
        proto.add_widths(w);
    session_proto::Color* oc_proto = proto.mutable_objectcolor();
    oc_proto->set_guid(objectcolor.guid());
    oc_proto->set_name(objectcolor.name);
    oc_proto->set_r(objectcolor.r);
    oc_proto->set_g(objectcolor.g);
    oc_proto->set_b(objectcolor.b);
    oc_proto->set_a(objectcolor.a);
    proto.set_color_mode(static_cast<int>(color_mode));
    return proto.SerializeAsString();
}

Mesh Mesh::pb_loads(const std::string& data) {
    session_proto::Mesh proto;
    proto.ParseFromString(data);
    Mesh mesh;
    if (!proto.guid().empty()) mesh.guid() = proto.guid();
    mesh.name = proto.name();
    for (const auto& [vkey, vdata] : proto.vertices()) {
        VertexData vd;
        vd.x = vdata.x();
        vd.y = vdata.y();
        vd.z = vdata.z();
        for (const auto& [k, v] : vdata.attributes())
            vd.attributes[k] = v;
        mesh.vertex[vkey] = vd;
    }
    for (const auto& [fkey, fdata] : proto.faces()) {
        std::vector<size_t> verts;
        for (uint64_t v : fdata.vertices())
            verts.push_back(v);
        mesh.face[fkey] = verts;
        for (const auto& [k, v] : fdata.attributes())
            mesh.facedata[fkey][k] = v;
        if (fdata.holes_size() > 0) {
            std::vector<std::vector<size_t>> rings;
            for (const session_proto::HoleRing& hole : fdata.holes()) {
                std::vector<size_t> ring;
                for (uint64_t v : hole.vertices()) ring.push_back(v);
                rings.push_back(ring);
            }
            mesh.face_holes[fkey] = rings;
        }
    }
    for (const auto& [fkey, tri_list] : proto.triangulation()) {
        std::vector<std::array<size_t, 3>> tris;
        const auto& vlist = tri_list.vertices();
        for (int i = 0; i + 2 < vlist.size(); i += 3)
            tris.push_back({static_cast<size_t>(vlist[i]), static_cast<size_t>(vlist[i+1]), static_cast<size_t>(vlist[i+2])});
        mesh.triangulation[fkey] = tris;
    }
    for (const session_proto::EdgeData& edata : proto.edge_data()) {
        const std::pair<size_t, size_t> key(static_cast<size_t>(edata.vertex1()), static_cast<size_t>(edata.vertex2()));
        for (const auto& [k, v] : edata.attributes())
            mesh.edgedata[key][k] = v;
    }
    for (const auto& [k, v] : proto.default_vertex_attributes())
        mesh.default_vertex_attributes[k] = v;
    for (const auto& [k, v] : proto.default_face_attributes())
        mesh.default_face_attributes[k] = v;
    for (const auto& [k, v] : proto.default_edge_attributes())
        mesh.default_edge_attributes[k] = v;
    mesh.pointcolors = colors_from_rgba(proto.pointcolors_rgba());
    mesh.facecolors = colors_from_rgba(proto.facecolors_rgba());
    mesh.linecolors = colors_from_rgba(proto.linecolors_rgba());
    for (double w : proto.widths())
        mesh.widths.push_back(w);
    const session_proto::Color& oc = proto.objectcolor();
    mesh.objectcolor = Color(oc.r(), oc.g(), oc.b(), oc.a());
    mesh.objectcolor.guid() = oc.guid();
    mesh.objectcolor.name = oc.name();
    mesh.color_mode = static_cast<ColorMode>(proto.color_mode());
    if (!mesh.vertex.empty()) mesh.max_vertex = mesh.vertex.rbegin()->first + 1;
    if (!mesh.face.empty()) mesh.max_face = mesh.face.rbegin()->first + 1;
    return mesh;
}

void Mesh::pb_dump(const std::string& filename) const {
    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

Mesh Mesh::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String Representation
// ═══════════════════════════════════════════════════════════════════════════

std::string Mesh::str() const {
    return fmt::format("Mesh(name={}, vertices={}, faces={})", name, number_of_vertices(), number_of_faces());
}

std::string Mesh::repr() const {
    return fmt::format("Mesh(\n  name={},\n  vertices={},\n  faces={},\n  edges={}\n)", name, number_of_vertices(), number_of_faces(), number_of_edges());
}

std::ostream& operator<<(std::ostream& os, const Mesh& mesh) {
    os << mesh.str();
    return os;
}

} // namespace session_cpp
