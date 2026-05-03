#include <filesystem>
#include <cmath>
#include "session.h"
#include "reciprocal.h"
#include "intersection.h"
#include "polyline.h"
#include "tolerance.h"
using namespace session_cpp;

struct BeamGeom {
    Mesh               mesh;
    std::vector<Point> side0;  // right side face outline (+right)
    std::vector<Point> side1;  // left  side face outline (-right)
};

// Build the side-face cut plane of the neighboring beam.
// endplane : from r.endplanes[ei][0 or 1] — encodes neighbor beam orientation.
//            z_axis ≈ dir_nb × up_nb  = neighbor's "right" direction.
// endpoint : the unextended center-line endpoint (= endplane origin position).
// dir_i    : direction of OUR beam.
// is_start : true → extension is in -dir_i; false → extension in +dir_i.
// beam_w   : cross-section width.
static Plane side_cut_plane(
    const Plane&  endplane,
    const Point&  endpoint,
    const Vector& dir_i,
    bool          is_start,
    double        beam_w,
    double        cut_offset = 0.0)
{
    Vector right_nb = endplane.z_axis().normalized();
    if (right_nb.is_zero())
        right_nb = endplane.x_axis().cross(Vector(0, 0, 1)).normalized();

    // Project our extension direction onto neighbor's right axis.
    // Extension goes in -dir_i (start) or +dir_i (end).
    double s = is_start ? 1.0 : -1.0;
    double side = s * (dir_i[0]*right_nb[0] + dir_i[1]*right_nb[1] + dir_i[2]*right_nb[2]);

    Point  face_pt;
    Vector face_normal;
    double half = beam_w * 0.5 + cut_offset;
    if (side > 0.0) {
        // Extension approaches from -right_nb side → left face is nearest
        face_pt    = Point(endpoint[0] - right_nb[0]*half,
                           endpoint[1] - right_nb[1]*half,
                           endpoint[2] - right_nb[2]*half);
        face_normal = Vector(-right_nb[0], -right_nb[1], -right_nb[2]);
    } else {
        // Extension approaches from +right_nb side → right face is nearest
        face_pt    = Point(endpoint[0] + right_nb[0]*half,
                           endpoint[1] + right_nb[1]*half,
                           endpoint[2] + right_nb[2]*half);
        face_normal = right_nb;
    }
    return Plane::from_point_normal(face_pt, face_normal);
}

// Build a rectangular box beam along `line`.
// up      = height direction (linePlane y_axis).
// extend  = extra length added symmetrically past start/end before cutting.
// cut_s/e = oblique cut planes at start/end (projects each corner along dir).
static BeamGeom make_beam(const Line& line, const Vector& up, double w, double h,
                           double extend,
                           const Plane& cut_s, const Plane& cut_e) {
    Vector dir = line.to_direction();

    Vector right = dir.cross(up);
    if (right.is_zero())
        right = dir.cross(Vector(0, 0, 1));
    right = right.normalized() * (w * 0.5);
    Vector n = up * (h * 0.5);

    Point s = Point(line.start()[0] - extend * dir[0],
                    line.start()[1] - extend * dir[1],
                    line.start()[2] - extend * dir[2]);
    Point e = Point(line.end()[0] + extend * dir[0],
                    line.end()[1] + extend * dir[1],
                    line.end()[2] + extend * dir[2]);

    auto corner = [](const Point& p, const Vector& r, int sr, const Vector& nn, int sn) {
        return Point(p[0] + sr*r[0] + sn*nn[0],
                     p[1] + sr*r[1] + sn*nn[1],
                     p[2] + sr*r[2] + sn*nn[2]);
    };

    auto cut = [&](const Point& p, const Plane& pl) -> Point {
        Point pt;
        Line ray = Line::from_points(p, Point(p[0]+dir[0], p[1]+dir[1], p[2]+dir[2]));
        if (Intersection::line_plane(ray, pl, pt, false))
            return pt;
        return p;
    };

    std::array<Point, 4> sc = {
        cut(corner(s, right, -1, n, -1), cut_s),  // 0 bot-left
        cut(corner(s, right, +1, n, -1), cut_s),  // 1 bot-right
        cut(corner(s, right, +1, n, +1), cut_s),  // 2 top-right
        cut(corner(s, right, -1, n, +1), cut_s),  // 3 top-left
    };
    std::array<Point, 4> ec = {
        cut(corner(e, right, -1, n, -1), cut_e),  // 4 bot-left
        cut(corner(e, right, +1, n, -1), cut_e),  // 5 bot-right
        cut(corner(e, right, +1, n, +1), cut_e),  // 6 top-right
        cut(corner(e, right, -1, n, +1), cut_e),  // 7 top-left
    };

    std::vector<Point> pts = {
        sc[0], sc[1], sc[2], sc[3],
        ec[0], ec[1], ec[2], ec[3],
    };
    std::vector<std::vector<size_t>> faces = {
        {0, 1, 2, 3},  // start cap  → normal: -dir
        {4, 7, 6, 5},  // end cap    → normal: +dir
        {0, 4, 5, 1},  // bottom     → normal: -up
        {1, 5, 6, 2},  // right      → normal: +right
        {2, 6, 7, 3},  // top        → normal: +up
        {3, 7, 4, 0},  // left       → normal: -right
    };

    BeamGeom bg;
    bg.mesh  = Mesh::from_vertices_and_faces(pts, faces);
    bg.side0 = {sc[1], sc[2], ec[2], ec[1]};  // right side face (+right)
    bg.side1 = {sc[3], sc[0], ec[0], ec[3]};  // left  side face (-right)
    return bg;
}

int main() {
    auto data = std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data";

    // ── Sinusoidal dome mesh ───────────────────────────────────────────────────
    constexpr int    nx = 12, ny = 10;
    constexpr double W  = 12.0, D = 10.0, h = 3.0;

    std::vector<Point> pts;
    pts.reserve((nx + 1) * (ny + 1));
    for (int j = 0; j <= ny; j++) {
        for (int i = 0; i <= nx; i++) {
            double x = W * i / nx;
            double y = D * j / ny;
            double z = h * std::sin(Tolerance::PI * x / W)
                         * std::sin(Tolerance::PI * y / D);
            pts.push_back(Point(x, y, z));
        }
    }

    std::vector<std::vector<size_t>> faces;
    faces.reserve(nx * ny);
    for (int j = 0; j < ny; j++) {
        for (int i = 0; i < nx; i++) {
            faces.push_back({
                (size_t)( j      * (nx + 1) + i    ),
                (size_t)( j      * (nx + 1) + i + 1),
                (size_t)((j + 1) * (nx + 1) + i + 1),
                (size_t)((j + 1) * (nx + 1) + i    ),
            });
        }
    }

    Mesh mesh = Mesh::from_vertices_and_faces(pts, faces);

    // U-edges connect vertices differing by 1 in flat index; V-edges by (nx+1).
    // Use this to stagger every second beam set by half depth along its normal.
    auto ekeys = mesh.edges();

    // ── Reciprocal frame ──────────────────────────────────────────────────────
    auto r = Reciprocal::from_mesh(mesh, 0.35, 1.4, true, 0.15);

    constexpr double beam_w = 0.10;
    constexpr double beam_h = 2.0 * beam_w;
    constexpr double extend     = 5.0 * beam_w;
    constexpr double cut_offset = 1.0 * beam_w;  // push cut plane past neighbour face

    // ── Session export ────────────────────────────────────────────────────────
    Session session("Reciprocal");
    auto g_mesh   = session.add_group("Mesh");
    auto g_beams  = session.add_group("Beams");
    auto g_side0  = session.add_group("Side0");
    auto g_side1  = session.add_group("Side1");
    auto g_top    = session.add_group("Top");
    auto g_bottom = session.add_group("Bottom");
    auto g_planes = session.add_group("EndPlanes");

    // Base mesh (grey)
    {
        auto m = std::make_shared<Mesh>(mesh);
        m->set_objectcolor(Color(210, 210, 210));
        session.add_mesh(m, g_mesh);
    }

    // Beams — cut at side face of the correct topological neighbor (from endplanes).
    // V-edges (index diff == nx+1) are shifted outward by half beam depth.
    for (int ei = 0; ei < (int)r.center.size(); ei++) {
        auto [u, v]   = ekeys[ei];
        bool is_v_edge = (size_t)std::abs((long long)u - (long long)v) == (size_t)(nx + 1);

        Line          ln  = r.center[ei];
        const Vector& up  = r.lineplanes[ei].y_axis();
        if (is_v_edge)
            ln += up * (beam_h * 0.5);
        const Vector  dir = ln.to_direction();

        Plane ps = side_cut_plane(r.endplanes[ei][0], ln.start(), dir, true,  beam_w, cut_offset);
        Plane pe = side_cut_plane(r.endplanes[ei][1], ln.end(),   dir, false, beam_w, cut_offset);

        auto bg = make_beam(ln, up, beam_w, beam_h, extend, ps, pe);

        bg.mesh.set_objectcolor(Color(160, 110, 55));
        session.add_mesh(std::make_shared<Mesh>(bg.mesh), g_beams);

        // Side 0 outline (right face, +right) — light amber
        {
            std::vector<Point> cl = bg.side0;
            cl.push_back(bg.side0[0]);
            auto pl = std::make_shared<Polyline>(cl);
            pl->linecolor = Color(220, 180, 100);
            pl->width = 1.5;
            session.add_polyline(pl, g_side0);
        }
        // Side 1 outline (left face, -right) — dark brown
        {
            std::vector<Point> cl = bg.side1;
            cl.push_back(bg.side1[0]);
            auto pl = std::make_shared<Polyline>(cl);
            pl->linecolor = Color(80, 50, 20);
            pl->width = 1.5;
            session.add_polyline(pl, g_side1);
        }
    }

    // Top lines (red)
    for (auto& l : r.top) {
        auto ln = std::make_shared<Line>(l);
        ln->linecolor = Color(200, 60, 60);
        ln->width = 1.0;
        session.add_line(ln, g_top);
    }

    // Bottom lines (green)
    for (auto& l : r.bottom) {
        auto ln = std::make_shared<Line>(l);
        ln->linecolor = Color(60, 160, 80);
        ln->width = 1.0;
        session.add_line(ln, g_bottom);
    }

    // End planes
    for (auto& [ps, pe] : r.endplanes) {
        auto p0 = std::make_shared<Plane>(ps); p0->width = 0.12;
        auto p1 = std::make_shared<Plane>(pe); p1->width = 0.12;
        session.add_plane(p0, g_planes);
        session.add_plane(p1, g_planes);
    }

    session.pb_dump((data / "reciprocal.pb").string());
    return 0;
}
