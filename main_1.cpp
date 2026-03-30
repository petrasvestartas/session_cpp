#include "session.h"
#include "intersection.h"
#include <cmath>
#include <iostream>

using namespace session_cpp;
static constexpr double PI2 = 6.283185307179586476;

static Polyline make_ngon(int n, double cx, double cy, double r) {
    std::vector<Point> pts;
    pts.reserve(n + 1);
    for (int i = 0; i < n; i++) {
        double a = PI2 * i / n;
        pts.push_back(Point(cx + r * std::cos(a), cy + r * std::sin(a), 0.0));
    }
    pts.push_back(pts[0]);
    return Polyline(pts);
}

static Polyline make_star(int n, double cx, double cy, double r_outer, double r_inner) {
    std::vector<Point> pts;
    pts.reserve(2 * n + 1);
    for (int i = 0; i < 2 * n; i++) {
        double a = PI2 * i / (2 * n);
        double r = (i % 2 == 0) ? r_outer : r_inner;
        pts.push_back(Point(cx + r * std::cos(a), cy + r * std::sin(a), 0.0));
    }
    pts.push_back(pts[0]);
    return Polyline(pts);
}

static Polyline make_L(double x, double y, double w, double h, double t) {
    return Polyline({
        Point(x, y, 0), Point(x+w, y, 0), Point(x+w, y+t, 0),
        Point(x+t, y+t, 0), Point(x+t, y+h, 0), Point(x, y+h, 0), Point(x, y, 0)
    });
}

static void add_case(Session& s, const Polyline& a, const Polyline& b, double offset_y) {
    Polyline pa = a, pb = b;
    pa.linecolor = Color::red();
    pb.linecolor = Color::blue();
    s.objects.polylines->push_back(std::make_shared<Polyline>(pa));
    s.objects.polylines->push_back(std::make_shared<Polyline>(pb));

    auto isect = Intersection::polyline_boolean(a, b, 0);
    auto uni   = Intersection::polyline_boolean(a, b, 1);
    auto diff  = Intersection::polyline_boolean(a, b, 2);

    for (auto& p : isect) { p.linecolor = Color::green();  s.objects.polylines->push_back(std::make_shared<Polyline>(p)); }
    for (auto& p : uni)   { p.linecolor = Color::orange(); s.objects.polylines->push_back(std::make_shared<Polyline>(p)); }
    for (auto& p : diff)  { p.linecolor = Color::cyan();   s.objects.polylines->push_back(std::make_shared<Polyline>(p)); }

    std::cout << "  isect=" << isect.size() << " uni=" << uni.size() << " diff=" << diff.size() << "\n";
}

int main() {
    Session s;

    // 1. Overlapping squares
    std::cout << "1. overlapping squares\n";
    add_case(s,
        Polyline({Point(-1,-1,0), Point(1,-1,0), Point(1,1,0), Point(-1,1,0), Point(-1,-1,0)}),
        Polyline({Point(0,0,0), Point(2,0,0), Point(2,2,0), Point(0,2,0), Point(0,0,0)}),
        0);

    // 2. Circle vs rectangle
    std::cout << "2. circle vs rectangle\n";
    add_case(s,
        make_ngon(64, 5.0, 0.0, 1.5),
        Polyline({Point(4,-0.5,0), Point(7,-0.5,0), Point(7,0.5,0), Point(4,0.5,0), Point(4,-0.5,0)}),
        0);

    // 3. Star vs circle
    std::cout << "3. star vs circle\n";
    add_case(s,
        make_star(5, 10.0, 0.0, 2.0, 0.8),
        make_ngon(32, 10.5, 0.5, 1.2),
        0);

    // 4. L-shape vs rectangle
    std::cout << "4. L-shape vs rectangle\n";
    add_case(s,
        make_L(15, -1, 3, 3, 1),
        Polyline({Point(15.5,-0.5,0), Point(18.5,-0.5,0), Point(18.5,1.5,0), Point(15.5,1.5,0), Point(15.5,-0.5,0)}),
        0);

    // 5. Two large overlapping circles (high vertex count)
    std::cout << "5. two 256-gon circles\n";
    add_case(s,
        make_ngon(256, 22.0, 0.0, 2.0),
        make_ngon(256, 23.0, 0.5, 2.0),
        0);

    // 6. Diamond vs triangle
    std::cout << "6. diamond vs triangle\n";
    add_case(s,
        Polyline({Point(28,0,0), Point(30,-2,0), Point(32,0,0), Point(30,2,0), Point(28,0,0)}),
        Polyline({Point(29,-2,0), Point(33,0,0), Point(29,2,0), Point(29,-2,0)}),
        0);

    // 7. Star vs star
    std::cout << "7. star vs star\n";
    add_case(s,
        make_star(6, 36.0, 0.0, 2.5, 1.0),
        make_star(5, 37.0, 0.5, 2.0, 0.8),
        0);

    // 8. Narrow rectangle vs wide rectangle (cross shape)
    std::cout << "8. cross shape\n";
    add_case(s,
        Polyline({Point(42,-2,0), Point(44,-2,0), Point(44,2,0), Point(42,2,0), Point(42,-2,0)}),
        Polyline({Point(40,-.5,0), Point(46,-.5,0), Point(46,.5,0), Point(40,.5,0), Point(40,-.5,0)}),
        0);

    // 9. Concave arrow vs circle
    std::cout << "9. concave arrow vs circle\n";
    add_case(s,
        Polyline({Point(49,0,0), Point(52,2,0), Point(51,0.5,0), Point(53,0.5,0),
                  Point(53,-0.5,0), Point(51,-0.5,0), Point(52,-2,0), Point(49,0,0)}),
        make_ngon(48, 51.5, 0.0, 1.5),
        0);

    // 10. Two 1000-gon circles (performance test)
    std::cout << "10. two 1000-gon circles\n";
    add_case(s,
        make_ngon(1000, 58.0, 0.0, 3.0),
        make_ngon(1000, 59.5, 0.0, 3.0),
        0);

    s.pb_dump("bool_all.pb");
    std::cout << "\nwrote bool_all.pb\n";
    return 0;
}
