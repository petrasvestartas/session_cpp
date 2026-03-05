#include "color.h"
#include "mesh.h"
#include "session.h"
#include <filesystem>

using namespace session_cpp;

int main() {

    Session session;

   


    std::vector<Line> lines = {
        Line::from_points(Point(4.948083, -0.149798, 1.00765),
                            Point(4.395544, -0.996413, 1.196018)),
        Line::from_points(Point(3.866593, 0.371225, 1.376346),
                            Point(4.567265, 0.584361, 1.137476)),
        Line::from_points(Point(3.915298, -0.157402, 1.359741),
                            Point(3.282977, -0.051356, 1.575309)),
        Line::from_points(Point(4.286215, -0.224964, 1.23329),
                            Point(3.607284, -0.987075, 1.464748)),
        Line::from_points(Point(3.744351, 0.971574, 1.41802),
                            Point(3.266367, 0.841359, 1.580972)),
        Line::from_points(Point(4.567265, 0.584361, 1.137476),
                            Point(4.948083, -0.149798, 1.00765)),
        Line::from_points(Point(4.395544, -0.996413, 1.196018),
                            Point(3.607284, -0.987075, 1.464748)),
        Line::from_points(Point(3.915298, -0.157402, 1.359741),
                            Point(4.286215, -0.224964, 1.23329)),
        Line::from_points(Point(3.282977, -0.051356, 1.575309),
                            Point(3.266367, 0.841359, 1.580972)),
        Line::from_points(Point(3.744351, 0.971574, 1.41802),
                            Point(3.866593, 0.371225, 1.376346)),
    };
    Mesh mesh = Mesh::from_lines(lines, true);
    
     session.add_mesh(std::make_shared<Mesh>(mesh));

    std::string filepath = (std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data" / "mesh_recheck.pb").string();
    session.pb_dump(filepath);

    return 0;
}
