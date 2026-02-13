#include "session.h"
#include "primitives.h"
#include <iostream>
#include <filesystem>

using namespace session_cpp;

int main() {
    Session session;

    auto u0 = NurbsCurve::create(false, 2, {
        Point(135, 17, 0), Point(131, 21, 0), Point(135, 26, 0)});
    auto u1 = NurbsCurve::create(false, 2, {
        Point(139.228777, 17.799797, 3.377253), Point(137.25766, 20.980203, 7.633725),
        Point(138.530306, 25.217183, 3.147742)});
    auto u2 = NurbsCurve::create(false, 2, {
        Point(140.519039, 17.722618, 3.491662), Point(142.519039, 22.722618, 7.491662),
        Point(141.023716, 25.344319, 3.431608)});
    auto u3 = NurbsCurve::create(false, 2, {
        Point(146, 16, 0), Point(150, 21.0, 7.0), Point(146, 27, 0)});

    auto v0 = NurbsCurve::create(false, 2, {
        Point(135, 17, 0), Point(140, 19, 7.0), Point(146, 16, 0)});
    auto v1 = NurbsCurve::create(false, 2, {
        Point(135, 26, 0), Point(140, 24, 7.0), Point(146, 27, 0)});

    session.add_curve(std::make_shared<NurbsCurve>(u0));
    session.add_curve(std::make_shared<NurbsCurve>(u1));
    session.add_curve(std::make_shared<NurbsCurve>(u2));
    session.add_curve(std::make_shared<NurbsCurve>(u3));
    session.add_curve(std::make_shared<NurbsCurve>(v0));
    session.add_curve(std::make_shared<NurbsCurve>(v1));

    // TODO: create_network not yet implemented
    // auto srf = Primitives::create_network({u0, u1, u2, u3}, {v0, v1});
    // session.add_surface(std::make_shared<NurbsSurface>(srf));

    std::string filepath = (std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data" / "network_surface.pb").string();
    session.pb_dump(filepath);
    std::cout << "Saved to: " << filepath << std::endl;

    return 0;
}
