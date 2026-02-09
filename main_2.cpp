#include "session.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
#include "mesh.h"
#include <fstream>
#include <cmath>

using namespace session_cpp;

int main() {
    auto uc0 = NurbsCurve::create(false, 2, {Point(10,9.569076,0), Point(5.5,9.569076,3.5), Point(1,9.569076,0)});
    auto uc1 = NurbsCurve::create(false, 2, {Point(10,16.569076,0), Point(5.5,16.569076,3.5), Point(1,16.569076,0)});
    auto vc0 = NurbsCurve::create(false, 3, {Point(1,9.569076,0), Point(1,11.569076,3.0), Point(1,14.569076,3.0), Point(1,16.569076,0)});
    auto vc1 = NurbsCurve::create(false, 2, {Point(4.236484,9.569076,1.612033), Point(3,13.069076,4.250144), Point(3.667141,16.569076,1.459684)});
    auto vc2 = NurbsCurve::create(false, 2, {Point(7.295129,16.569076,1.471513), Point(8,13.069076,4.250144), Point(6.99265,9.569076,1.557456)});
    auto vc3 = NurbsCurve::create(false, 3, {Point(10,9.569076,0), Point(10,11.569076,3), Point(10,14.569076,3), Point(10,16.569076,0)});

    NurbsSurface srf = NurbsSurface::create_network({uc0, uc1}, {vc0, vc1, vc2, vc3});

    std::cerr << "cv_count(0)=" << srf.cv_count(0) << " cv_count(1)=" << srf.cv_count(1)
              << " deg(0)=" << srf.degree(0) << " deg(1)=" << srf.degree(1) << std::endl;

    auto check_crv = [&](const char* name, auto eval_srf, const NurbsCurve& crv) {
        std::cerr << "=== " << name << " ===" << std::endl;
        double max_err = 0;
        for (int k = 0; k <= 10; k++) {
            double t = k / 10.0;
            auto [t0, t1] = crv.domain();
            Point ps = eval_srf(t);
            Point pc = crv.point_at(t0 + t * (t1 - t0));
            double err = std::sqrt((ps[0]-pc[0])*(ps[0]-pc[0])+(ps[1]-pc[1])*(ps[1]-pc[1])+(ps[2]-pc[2])*(ps[2]-pc[2]));
            if (err > max_err) max_err = err;
        }
        std::cerr << "  max_err=" << max_err << std::endl;
    };
    check_crv("uc0 (v=0)", [&](double u){ return srf.point_at(u, 0.0); }, uc0);
    check_crv("uc1 (v=1)", [&](double u){ return srf.point_at(u, 1.0); }, uc1);
    check_crv("vc0 (u=1)", [&](double v){ return srf.point_at(1.0, v); }, vc0);
    check_crv("vc3 (u=0)", [&](double v){ return srf.point_at(0.0, v); }, vc3);
    check_crv("vc1 interior u=0.672", [&](double v){ return srf.point_at(0.672021, v); }, vc1);
    auto vc2r = vc2; vc2r.reverse(); vc2r.set_domain(0.0, 1.0);
    check_crv("vc2 interior u=0.317 (rev)", [&](double v){ return srf.point_at(0.317346, v); }, vc2r);

    auto ku = srf.get_knots(0);
    auto kv = srf.get_knots(1);
    std::cerr << "knots_u(" << ku.size() << "): ";
    for (auto k : ku) std::cerr << k << " ";
    std::cerr << std::endl;
    std::cerr << "knots_v(" << kv.size() << "): ";
    for (auto k : kv) std::cerr << k << " ";
    std::cerr << std::endl;
    std::cerr << "cv[0,0]=" << srf.get_cv(0,0) << std::endl;
    std::cerr << "cv[15,2]=" << srf.get_cv(15,2) << std::endl;
    std::cerr << "raw m_cv[186..189]=" << srf.m_cv[186] << " " << srf.m_cv[187] << " " << srf.m_cv[188] << std::endl;
    std::cerr << "stride[0]=" << srf.m_cv_stride[0] << " stride[1]=" << srf.m_cv_stride[1] << std::endl;
    int real_idx = 15 * srf.m_cv_stride[0] + 2 * srf.m_cv_stride[1];
    std::cerr << "real index for cv(15,2)=" << real_idx << " values=" << srf.m_cv[real_idx] << " " << srf.m_cv[real_idx+1] << " " << srf.m_cv[real_idx+2] << std::endl;
    std::cerr << "cv[31,3]=" << srf.get_cv(31,3) << std::endl;
    std::cerr << "REF point_at(0,0)=" << srf.point_at(0,0) << std::endl;
    std::cerr << "REF point_at(0.5,0)=" << srf.point_at(0.5,0) << std::endl;
    std::cerr << "REF point_at(0.5,0.5)=" << srf.point_at(0.5,0.5) << std::endl;
    std::cerr << "REF point_at(0,0.5)=" << srf.point_at(0,0.5) << std::endl;
    std::cerr << "REF point_at(1,1)=" << srf.point_at(1,1) << std::endl;
    Mesh m = srf.mesh();

    Session session("create_network");
    for (auto& c : {uc0, uc1, vc0, vc1, vc2, vc3})
        session.add_curve(std::make_shared<NurbsCurve>(c));
    session.add_surface(std::make_shared<NurbsSurface>(srf));
    session.add_mesh(std::make_shared<Mesh>(m));
    session.pb_dump("C:/pc/3_code/code_rust/session/session_data/create_network2.pb");

    return 0;
}
