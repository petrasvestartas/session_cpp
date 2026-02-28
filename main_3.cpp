#include "session.h"
#include "mesh.h"
#include "line.h"
#include "polyline.h"
#include "point.h"
#include <cmath>
#include <filesystem>

using namespace session_cpp;
static constexpr double TAU = 6.28318530717958647692;

// n-gon centred at (cx,cy,cz), radius r, in XY plane
static std::vector<Point> ngon(double cx, double cy, double cz, double r, int n, double phase = 0) {
    std::vector<Point> pts;
    for (int i = 0; i < n; ++i) {
        double a = TAU * i / n + phase;
        pts.push_back({cx + r * std::cos(a), cy + r * std::sin(a), cz});
    }
    return pts;
}

int main() {
    Session session("mesh_cdt");
    const double cx = 16.0;  // column spacing
    const double ry = 15.0;  // row spacing
    auto add = [&](Mesh& m, const char* name) {
        m.name = name;
        session.add_mesh(std::make_shared<Mesh>(m));
    };

    // ==========================================================
    // ROW 0  —  baseline meshes (box / from_polylines / from_lines)
    // ==========================================================

    {
        Mesh m = Mesh::create_box(8.0, 8.0, 8.0);
        add(m, "box 8x8x8");
    }

    {
        double bx = cx, by = 0;
        Mesh m = Mesh::from_polylines({
            {{bx+1.28955,by+0,       1.127558},{bx+0.85791,by+0,       0.225512},
             {bx+0.64209,by-0.866025,-0.225512},{bx+0.85791,by-1.732051,0.225512},
             {bx+1.458565,by-1.732051,1.127558},{bx+1.50537,by-0.866025,1.578581}},
            {{bx+0.64209,by+0.866025,-0.225512},{bx+0.114274,by+0.866025,-0.686294},
             {bx-0.00537,by+0,-1.578581},{bx+0.21045,by-0.866025,-1.127558},
             {bx+0.64209,by-0.866025,-0.225512},{bx+0.85791,by+0,0.225512}},
            {{bx+1.28955,by+1.732051,1.127558},{bx+0.85791,by+1.732051,0.225512},
             {bx+0.64209,by+0.866025,-0.225512},{bx+0.85791,by+0,0.225512},
             {bx+1.28955,by+0,1.127558},{bx+1.853404,by+0.866025,1.578581}},
        });
        add(m, "from_polylines");
    }

    {
        double bx = 2*cx, by = 0;
        Mesh m = Mesh::from_lines({
            Line::from_points({bx+4.948083,by-0.149798,1.00765 },{bx+4.395544,by-0.996413,1.196018}),
            Line::from_points({bx+3.866593,by+0.371225,1.376346},{bx+4.567265,by+0.584361,1.137476}),
            Line::from_points({bx+3.915298,by-0.157402,1.359741},{bx+3.282977,by-0.051356,1.575309}),
            Line::from_points({bx+4.286215,by-0.224964,1.23329 },{bx+3.607284,by-0.987075,1.464748}),
            Line::from_points({bx+3.744351,by+0.971574,1.41802 },{bx+3.266367,by+0.841359,1.580972}),
            Line::from_points({bx+4.567265,by+0.584361,1.137476},{bx+4.948083,by-0.149798,1.00765 }),
            Line::from_points({bx+4.395544,by-0.996413,1.196018},{bx+3.607284,by-0.987075,1.464748}),
            Line::from_points({bx+3.915298,by-0.157402,1.359741},{bx+4.286215,by-0.224964,1.23329 }),
            Line::from_points({bx+3.282977,by-0.051356,1.575309},{bx+3.266367,by+0.841359,1.580972}),
            Line::from_points({bx+3.744351,by+0.971574,1.41802 },{bx+3.866593,by+0.371225,1.376346}),
        }, true);
        add(m, "from_lines");
    }

    // ==========================================================
    // ROW 1  —  CDT: simple / non-convex polygons (no holes)
    // ==========================================================

    {   // 16-gon — convex, many sides
        double bx = 0, by = ry;
        Mesh m = Mesh::from_polygon_with_holes({ngon(bx+6,by+6,0,6.0,16)}, true);
        add(m, "16-gon convex");
    }

    {   // L-shape — non-convex, single polygon
        double bx = cx, by = ry;
        Mesh m = Mesh::from_polygon_with_holes({{
            {bx+0,by+0,0},{bx+10,by+0,0},{bx+10,by+4,0},
            {bx+4,by+4,0},{bx+4,by+10,0},{bx+0,by+10,0},
        }}, true);
        add(m, "L-shape (no holes)");
    }

    {   // T-shape — non-convex, single polygon
        double bx = 2*cx, by = ry;
        Mesh m = Mesh::from_polygon_with_holes({{
            {bx+0,by+3,0},{bx+10,by+3,0},{bx+10,by+5,0},{bx+7,by+5,0},
            {bx+7,by+10,0},{bx+3,by+10,0},{bx+3,by+5,0},{bx+0,by+5,0},
        }}, true);
        add(m, "T-shape (no holes)");
    }

    {   // 5-pointed star — highly non-convex
        double bx = 3*cx, by = ry;
        std::vector<Point> star;
        for (int i = 0; i < 5; ++i) {
            double ao = TAU*i/5 - TAU/4;
            double ai = ao + TAU/10;
            star.push_back({bx+6+6.0*std::cos(ao), by+6+6.0*std::sin(ao), 0});
            star.push_back({bx+6+2.2*std::cos(ai), by+6+2.2*std::sin(ai), 0});
        }
        Mesh m = Mesh::from_polygon_with_holes({star}, true);
        add(m, "5-star (no holes)");
    }

    // ==========================================================
    // ROW 2  —  CDT: polygons WITH holes
    // ==========================================================

    {   // square outer + square hole (from test)
        double bx = 0, by = 2*ry;
        Mesh m = Mesh::from_polygon_with_holes({
            {{bx+1,by+1,0},{bx+4,by+1,0},{bx+4,by+4,0},{bx+1,by+4,0}},
            {{bx+0,by+0,0},{bx+5,by+0,0},{bx+5,by+5,0},{bx+0,by+5,0}},
        }, true);
        add(m, "square with square hole");
    }

    {   // 12-gon outer + 3 hex holes
        double bx = cx, by = 2*ry;
        Mesh m = Mesh::from_polygon_with_holes({
            ngon(bx+6,by+6,0,6.0,12),
            ngon(bx+4,by+7,0,1.4,6),
            ngon(bx+6,by+4,0,1.4,6),
            ngon(bx+8,by+7,0,1.4,6),
        }, true);
        add(m, "12-gon with 3 hex holes");
    }

    {   // rectangle outer + 2 rectangular holes
        double bx = 2*cx, by = 2*ry;
        Mesh m = Mesh::from_polygon_with_holes({
            {{bx+1.0,by+1.0,0},{bx+3.5,by+1.0,0},{bx+3.5,by+5.0,0},{bx+1.0,by+5.0,0}},
            {{bx+4.5,by+1.0,0},{bx+7.5,by+1.0,0},{bx+7.5,by+5.0,0},{bx+4.5,by+5.0,0}},
            {{bx+0,  by+0,  0},{bx+8.5,by+0,  0},{bx+8.5,by+6.0,0},{bx+0,  by+6.0,0}},
        }, true);
        add(m, "rectangle with 2 holes");
    }

    {   // L-shape outer + 2 octagonal holes
        double bx = 3*cx, by = 2*ry;
        Mesh m = Mesh::from_polygon_with_holes({
            {{bx+0,by+0,0},{bx+10,by+0,0},{bx+10,by+4,0},
             {bx+4,by+4,0},{bx+4,by+10,0},{bx+0,by+10,0}},
            ngon(bx+2,  by+2,  0, 1.3, 8),
            ngon(bx+7.5,by+2,  0, 1.3, 8),
        }, true);
        add(m, "L-shape with 2 holes");
    }

    {   // exact test data — 3D tilted polygon with 2 holes
        Mesh m = Mesh::from_polygon_with_holes({
            {{ 8.940934, 0.917382, 0.049546},{ 8.930493, 1.364580, 0.251429},
             { 8.954508, 1.595448, 0.346958},{ 9.457671, 1.821395, 0.298639},
             { 9.717078, 1.014296,-0.136839},{ 9.363048, 0.915340,-0.076160},
             { 9.333270, 0.459713,-0.269899},{ 9.065708, 0.635281,-0.112748}},
            {{ 7.494779,-0.556523,-0.178103},{ 6.542877, 0.148384, 0.416685},
             { 6.967337, 2.119511, 1.167431},{11.204553, 2.961749, 0.289102},
             { 9.658416, 0.465135,-0.363618},{10.247775,-1.032727,-1.203717}},
            {{ 7.922105, 0.548716, 0.186877},{ 7.410178, 0.844297, 0.469625},
             { 7.408889, 1.185147, 0.621527},{ 7.885956, 1.424645, 0.586947},
             { 8.178727, 1.329960, 0.458299},{ 8.307609, 0.882540, 0.221300},
             { 7.950364, 0.924872, 0.345738}},
        }, true);
        add(m, "3D polygon with 2 holes");
    }

    // ==========================================================
    // ROW 3  —  from_polygon_with_holes_many  (6 panels)
    // ==========================================================

    {
        std::vector<std::vector<std::vector<Point>>> inputs;
        for (int i = 0; i < 6; ++i) {
            double bx = i*12.0, by = 3*ry;
            inputs.push_back({
                ngon(bx+6, by+6, 0, 3.5, 6),           // hex hole
                {{bx+0,by+0,0},{bx+12,by+0,0},{bx+12,by+12,0},{bx+0,by+12,0}},  // outer
            });
        }
        auto meshes = Mesh::from_polygon_with_holes_many(inputs);
        for (int i = 0; i < (int)meshes.size(); ++i) {
            meshes[i].name = "panel_hex_hole_" + std::to_string(i);
            session.add_mesh(std::make_shared<Mesh>(meshes[i]));
        }
    }

    // ==========================================================
    // ROW 4  —  Loft with CDT caps
    // ==========================================================

    {   // rectangle → smaller rectangle (CDT caps)
        double bx = 0, by = 4*ry;
        Polyline b(std::vector<Point>{{bx+0,by+0,0},{bx+10,by+0,0},{bx+10,by+8,0},{bx+0,by+8,0},{bx+0,by+0,0}});
        Polyline t(std::vector<Point>{{bx+2,by+2,6},{bx+8, by+2,6},{bx+8, by+6,6},{bx+2,by+6,6},{bx+2,by+2,6}});
        Mesh m = Mesh::loft({b}, {t}, true);
        add(m, "loft rect cap (CDT)");
    }

    {   // L-shape bottom → top: CDT triangulates L-shaped caps
        double bx = cx, by = 4*ry;
        auto mk_l = [](double bx, double by, double z) {
            return Polyline(std::vector<Point>{
                {bx+0,by+0,z},{bx+10,by+0,z},{bx+10,by+4,z},
                {bx+4,by+4,z},{bx+4,by+10,z},{bx+0,by+10,z},{bx+0,by+0,z}});
        };
        Mesh m = Mesh::loft({mk_l(bx,by,0)}, {mk_l(bx,by,5)}, true);
        add(m, "loft L-cap (CDT)");
    }

    {   // T-shape bottom → top: CDT triangulates T-shaped caps
        double bx = 2*cx, by = 4*ry;
        auto mk_t = [](double bx, double by, double z) {
            return Polyline(std::vector<Point>{
                {bx+0,by+3,z},{bx+10,by+3,z},{bx+10,by+5,z},{bx+7,by+5,z},
                {bx+7,by+10,z},{bx+3,by+10,z},{bx+3,by+5,z},{bx+0,by+5,z},{bx+0,by+3,z}});
        };
        Mesh m = Mesh::loft({mk_t(bx,by,0)}, {mk_t(bx,by,5)}, true);
        add(m, "loft T-cap (CDT)");
    }

    {   // 3-profile loft from test (outer + 2 inner polylines, CDT on caps)
        double offx = 3*cx - 12.248787, offy = 4*ry + 1.032727;
        std::vector<Polyline> bottom = {
            Polyline(std::vector<Point>{
                {13.20069 +offx,-0.556523+offy,-0.178103},{12.248787+offx, 0.148384+offy, 0.416685},
                {12.673247+offx, 2.119511+offy, 1.167431},{16.910464+offx, 2.961749+offy, 0.289102},
                {15.364327+offx, 0.465135+offy,-0.363618},{15.953685+offx,-1.032727+offy,-1.203717},
                {13.20069 +offx,-0.556523+offy,-0.178103},
            }),
            Polyline(std::vector<Point>{
                {14.646845+offx, 0.917382+offy, 0.049546},{14.636404+offx, 1.364580+offy, 0.251429},
                {14.660418+offx, 1.595448+offy, 0.346958},{15.163581+offx, 1.821395+offy, 0.298639},
                {15.422988+offx, 1.014296+offy,-0.136839},{15.068958+offx, 0.915340+offy,-0.076160},
                {15.039180+offx, 0.459713+offy,-0.269899},{14.771618+offx, 0.635281+offy,-0.112748},
                {14.646845+offx, 0.917382+offy, 0.049546},
            }),
            Polyline(std::vector<Point>{
                {13.628016+offx, 0.548716+offy, 0.186877},{13.116088+offx, 0.844297+offy, 0.469625},
                {13.114799+offx, 1.185147+offy, 0.621527},{13.591866+offx, 1.424645+offy, 0.586947},
                {13.884637+offx, 1.329960+offy, 0.458299},{14.013519+offx, 0.882540+offy, 0.221300},
                {13.656275+offx, 0.924872+offy, 0.345738},{13.628016+offx, 0.548716+offy, 0.186877},
            }),
        };
        std::vector<Polyline> top = {
            Polyline(std::vector<Point>{
                {13.375135+offx,-0.818817+offy, 0.411936},{12.423233+offx,-0.113909+offy, 1.006724},
                {12.847692+offx, 1.857217+offy, 1.757470},{17.084909+offx, 2.699455+offy, 0.879141},
                {15.538772+offx, 0.202841+offy, 0.226421},{16.128130+offx,-1.295021+offy,-0.613678},
                {13.375135+offx,-0.818817+offy, 0.411936},
            }),
            Polyline(std::vector<Point>{
                {14.821290+offx, 0.655088+offy, 0.639585},{14.810849+offx, 1.102286+offy, 0.841468},
                {14.834864+offx, 1.333154+offy, 0.936997},{15.338026+offx, 1.559101+offy, 0.888678},
                {15.597433+offx, 0.752002+offy, 0.453200},{15.243404+offx, 0.653046+offy, 0.513879},
                {15.213626+offx, 0.197419+offy, 0.320140},{14.946063+offx, 0.372987+offy, 0.477291},
                {14.821290+offx, 0.655088+offy, 0.639585},
            }),
            Polyline(std::vector<Point>{
                {13.802461+offx, 0.286422+offy, 0.776916},{13.290534+offx, 0.582003+offy, 1.059664},
                {13.289245+offx, 0.922853+offy, 1.211566},{13.766312+offx, 1.162351+offy, 1.176986},
                {14.059082+offx, 1.067666+offy, 1.048338},{14.187964+offx, 0.620246+offy, 0.811339},
                {13.830720+offx, 0.662578+offy, 0.935777},{13.802461+offx, 0.286422+offy, 0.776916},
            }),
        };
        Mesh m = Mesh::loft(bottom, top, true);
        add(m, "loft 3-profile (test data)");
    }

    // ==========================================================
    // ROW 5  —  loft_many: 6 closed boxes of increasing height
    // ==========================================================

    {
        std::vector<std::pair<std::vector<Polyline>, std::vector<Polyline>>> pairs;
        for (int i = 0; i < 6; ++i) {
            double bx = i*12.0, by = 5*ry, h = 2.0 + i*1.5;
            Polyline b(std::vector<Point>{
                {bx+0,by+0,0},{bx+8,by+0,0},{bx+8,by+8,0},{bx+0,by+8,0},{bx+0,by+0,0}});
            Polyline t(std::vector<Point>{
                {bx+0,by+0,h},{bx+8,by+0,h},{bx+8,by+8,h},{bx+0,by+8,h},{bx+0,by+0,h}});
            pairs.push_back({{b}, {t}});
        }
        auto meshes = Mesh::loft_many(pairs);
        for (int i = 0; i < (int)meshes.size(); ++i) {
            meshes[i].name = "loft_box_" + std::to_string(i);
            session.add_mesh(std::make_shared<Mesh>(meshes[i]));
        }
    }

    std::string filepath = (std::filesystem::path(__FILE__).parent_path().parent_path()
                            / "session_data" / "mesh_cdt.pb").string();
    session.pb_dump(filepath);
    return 0;
}
