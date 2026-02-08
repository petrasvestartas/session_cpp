#include "session.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
#include "primitives.h"
#include "mesh.h"
#include "line.h"
#include <fstream>
#include <cmath>

using namespace session_cpp;

int main() {
    std::cout << "main_3.cpp" << std::endl;

    std::vector<Point> pts_a = {Point(3,0,0), Point(-2,0,5)};
    std::vector<Point> pts_b = {Point(3,5,5), Point(-2,5,0)};
    NurbsCurve crvA = NurbsCurve::create(false, 1, pts_a);
    NurbsCurve crvB = NurbsCurve::create(false, 1, pts_b);
    NurbsSurface s = NurbsSurface::create_ruled(crvA, crvB);

    // Mesh 180 - lowest quality, 5 - highest quality
    Mesh m = s.mesh(45);

     auto [rd, ruv] = s.divide_by_count(4, 4);


    std::vector<Point> pts;
    for (int i = 0; i < (int)rd.size(); i++)
        for (int j = 0; j < (int)rd[i].size(); j++)
            pts.push_back(rd[i][j]);

    std::vector<Vector> normals;
    for (int i = 0; i < (int)ruv.size(); i++)
        for (int j = 0; j < (int)ruv[i].size(); j++)
            normals.push_back(s.normal_at(ruv[i][j].first, ruv[i][j].second));

    std::vector<std::pair<double,double>> uvs;
    for (int i = 0; i < (int)ruv.size(); i++)
        for (int j = 0; j < (int)ruv[i].size(); j++)
            uvs.push_back(ruv[i][j]);

    // Create normal lines at each divide point
    std::vector<std::shared_ptr<Line>> normal_lines;
    for (size_t i = 0; i < pts.size(); i++) {
        Point end(pts[i][0] + normals[i][0], pts[i][1] + normals[i][1], pts[i][2] + normals[i][2]);
        auto line = std::make_shared<Line>(Line::from_points(pts[i], end));
        line->linecolor = Color::red();
        normal_lines.push_back(line);
    }

    // Copy (duplicates everything except guid)
    NurbsSurface scopy = s;

    Session session("nurbs_meshing_3");
    session.add_surface(std::make_shared<NurbsSurface>(scopy));
    for (auto& line : normal_lines)
        session.add_line(line);
    session.json_dump("C:/pc/3_code/code_rust/session/session_data/nurbs_meshing_3.json");    


    return 0;
}
