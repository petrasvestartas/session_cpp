#include "src/nurbscurve.h"
#include "src/nurbssurface.h"
#include "src/triangulation_nurbs.h"
#include "src/mesh.h"
#include "src/point.h"
#include <iostream>
#include <iomanip>

using namespace session_cpp;

int main() {

    // --- Ruled Surface (Hypar) ---
    std::cout << "=== Ruled Surface ===" << std::endl;
    std::vector<Point> pts_a = {Point(3,0,0), Point(-2,0,5)};
    std::vector<Point> pts_b = {Point(3,5,5), Point(-2,5,0)};
    NurbsCurve crvA = NurbsCurve::create(false, 1, pts_a);
    NurbsCurve crvB = NurbsCurve::create(false, 1, pts_b);
    NurbsSurface ruled = NurbsSurface::create_ruled(crvA, crvB);
    std::cout << ruled.str() << std::endl;

    auto [rd, ruv] = ruled.divide_by_count(4, 4);

    std::cout << std::setprecision(15);
    std::cout << "\nPoints:" << std::endl;
    std::vector<Point> pts;
    for (int i = 0; i < (int)rd.size(); i++)
        for (int j = 0; j < (int)rd[i].size(); j++) {
            pts.push_back(rd[i][j]);
            std::cout << rd[i][j][0] << ", " << rd[i][j][1] << ", " << rd[i][j][2] << std::endl;
        }

    std::cout << "\nNormals:" << std::endl;
    std::vector<Vector> normals;
    for (int i = 0; i < (int)ruv.size(); i++)
        for (int j = 0; j < (int)ruv[i].size(); j++) {
            Vector n = ruled.normal_at(ruv[i][j].first, ruv[i][j].second);
            normals.push_back(n);
            std::cout << n[0] << ", " << n[1] << ", " << n[2] << std::endl;
        }

    std::cout << "\nUV:" << std::endl;
    std::vector<std::pair<double,double>> uvs;
    for (int i = 0; i < (int)ruv.size(); i++)
        for (int j = 0; j < (int)ruv[i].size(); j++) {
            uvs.push_back(ruv[i][j]);
            std::cout << "{" << ruv[i][j].first << ", " << ruv[i][j].second << "}" << std::endl;
        }

    // --- Loft Surface ---
    std::cout << "\n=== Loft Surface ===" << std::endl;
    std::vector<Point> pts_c = {Point(0,0,0), Point(1,0,0), Point(2,0,0), Point(3,0,0)};
    std::vector<Point> pts_d = {Point(0,1,1), Point(1,1,2), Point(2,1,2), Point(3,1,1)};
    std::vector<Point> pts_e = {Point(0,2,0), Point(1,2,1), Point(2,2,1), Point(3,2,0)};
    NurbsCurve crvC = NurbsCurve::create(false, 3, pts_c);
    NurbsCurve crvD = NurbsCurve::create(false, 3, pts_d);
    NurbsCurve crvE = NurbsCurve::create(false, 3, pts_e);
    NurbsSurface loft = NurbsSurface::create_loft({crvC, crvD, crvE}, 2);
    std::cout << loft.str() << std::endl;

    auto [ld, luv] = loft.divide_by_count(4, 4);
    for (int i = 0; i < (int)ld.size(); i++)
        for (int j = 0; j < (int)ld[i].size(); j++)
            std::cout << ld[i][j] << std::endl;

    // --- Revolve Surface ---
    std::cout << "\n=== Revolve Surface ===" << std::endl;
    std::vector<Point> pts_profile = {Point(1,0,0), Point(1,0,1), Point(2,0,2)};
    NurbsCurve profile = NurbsCurve::create(false, 2, pts_profile);
    NurbsSurface revolve = NurbsSurface::create_revolve(profile, Point(0,0,0), Vector(0,0,1));
    std::cout << revolve.str() << std::endl;

    auto [rev_d, rev_uv] = revolve.divide_by_count(8, 4);
    for (int i = 0; i < (int)rev_d.size(); i++)
        for (int j = 0; j < (int)rev_d[i].size(); j++)
            std::cout << rev_d[i][j] << std::endl;

    // --- Sweep1 Surface ---
    std::cout << "\n=== Sweep1 Surface ===" << std::endl;
    std::vector<Point> pts_rail = {Point(0,0,0), Point(2,0,1), Point(4,0,0)};
    std::vector<Point> pts_sect = {Point(0,-1,0), Point(0,0,1), Point(0,1,0)};
    NurbsCurve rail = NurbsCurve::create(false, 2, pts_rail);
    NurbsCurve sect = NurbsCurve::create(false, 2, pts_sect);
    NurbsSurface sweep1 = NurbsSurface::create_sweep1(rail, sect);
    std::cout << sweep1.str() << std::endl;

    auto [s1d, s1uv] = sweep1.divide_by_count(4, 4);
    for (int i = 0; i < (int)s1d.size(); i++)
        for (int j = 0; j < (int)s1d[i].size(); j++)
            std::cout << s1d[i][j] << std::endl;

    // --- Sweep2 Surface ---
    std::cout << "\n=== Sweep2 Surface ===" << std::endl;
    std::vector<Point> pts_r1 = {Point(0,0,0), Point(2,0,1), Point(4,0,0)};
    std::vector<Point> pts_r2 = {Point(0,3,0), Point(2,3,2), Point(4,3,0)};
    std::vector<Point> pts_s2 = {Point(0,0,0), Point(0,1.5,1), Point(0,3,0)};
    NurbsCurve rail1 = NurbsCurve::create(false, 2, pts_r1);
    NurbsCurve rail2 = NurbsCurve::create(false, 2, pts_r2);
    NurbsCurve prof2 = NurbsCurve::create(false, 2, pts_s2);
    NurbsSurface sweep2 = NurbsSurface::create_sweep2(rail1, rail2, prof2);
    std::cout << sweep2.str() << std::endl;

    auto [s2d, s2uv] = sweep2.divide_by_count(4, 4);
    for (int i = 0; i < (int)s2d.size(); i++)
        for (int j = 0; j < (int)s2d[i].size(); j++)
            std::cout << s2d[i][j] << std::endl;

    // --- Planar Surface ---
    std::cout << "\n=== Planar Surface ===" << std::endl;
    std::vector<Point> pts_planar = {Point(0,0,0), Point(3,1,0), Point(5,0.5,0), Point(6,3,0), Point(4,5,0), Point(1,4,0), Point(0,0,0)};
    NurbsCurve boundary = NurbsCurve::create(false, 3, pts_planar);
    NurbsSurface planar = NurbsSurface::create_planar({boundary});
    std::cout << planar.str() << std::endl;
    std::cout << "is_trimmed: " << planar.is_trimmed() << std::endl;

    auto [bd_pts, bd_params] = boundary.divide_by_count(20, true);
    std::cout << "\nBoundary divide points: " << bd_pts.size() << std::endl;

    auto [pd, puv] = planar.divide_by_count(4, 4);
    std::cout << "Planar surface grid: " << pd.size() << "x" << pd[0].size() << std::endl;

    NurbsTriangulation tri(planar);
    Mesh planar_mesh = tri.mesh();
    std::cout << "Trimmed mesh: " << planar_mesh.number_of_faces() << " faces, " << planar_mesh.number_of_vertices() << " vertices" << std::endl;

    return 0;
}
