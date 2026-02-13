#include "session.h"
#include "primitives.h"
#include "knot.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <filesystem>

using namespace session_cpp;

int main() {
    Session session;

    auto pdist = [](const Point& a, const Point& b) {
        double dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2];
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    };

    // === Test 1: Open curve (verified working) ===
    {
        std::vector<Point> points = {
            {14, 9, 0}, {21, 22, 0}, {26, 10, 0}, {35, 19, 0}, {41, 13, 0}
        };
        NurbsCurve curve = Primitives::create_interpolated(points, CurveKnotStyle::Chord);
        std::cout << "=== OPEN CURVE ===" << std::endl;
        std::cout << "degree: " << curve.degree() << " cv_count: " << curve.cv_count() << std::endl;

        int n = (int)points.size();
        std::vector<double> pts_flat(n * 3);
        for (int i = 0; i < n; i++) {
            pts_flat[i*3] = points[i][0]; pts_flat[i*3+1] = points[i][1]; pts_flat[i*3+2] = points[i][2];
        }
        auto params = knot::compute_parameters(pts_flat.data(), n, 3, CurveKnotStyle::Chord);
        double max_err = 0;
        for (int i = 0; i < n; i++) {
            Point p = curve.point_at(params[i]);
            double err = pdist(p, points[i]);
            if (err > max_err) max_err = err;
        }
        std::cout << "Max interpolation error: " << std::scientific << std::setprecision(4)
                  << max_err << (max_err < 1e-10 ? " PASS" : " FAIL") << std::endl;
        session.add_curve(std::make_shared<NurbsCurve>(curve));
    }

    // === Test 2: Periodic (closed) curve ===
    {
        // 10 unique points forming a closed loop (no duplicate endpoint)
        std::vector<Point> points = {
            {4, 20, 0}, {-2, 20, 0}, {-2, 25, 0}, {-3, 28, 0}, {-10, 28, 0},
            {-10, 21, 0}, {-13, 16, 0}, {-8, 14, 0}, {-6, 11, 0}, {0, 15, 0}
        };
        NurbsCurve curve = Primitives::create_interpolated(points, CurveKnotStyle::ChordPeriodic);
        std::cout << "\n=== PERIODIC CURVE ===" << std::endl;
        std::cout << "degree: " << curve.degree() << " cv_count: " << curve.cv_count()
                  << " knot_count: " << curve.knot_count() << std::endl;

        // Print knots
        std::cout << "Knots:";
        for (int i = 0; i < curve.knot_count(); i++)
            std::cout << " " << std::fixed << std::setprecision(4) << curve.knot(i);
        std::cout << std::endl;

        // Print CVs
        std::cout << "CVs:" << std::endl;
        for (int i = 0; i < curve.cv_count(); i++) {
            Point p = curve.get_cv(i);
            std::cout << "  " << i << ": " << std::fixed << std::setprecision(6)
                      << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
        }

        // Verify: curve passes through all points
        int n = (int)points.size();
        // Compute periodic params
        std::vector<double> params(n + 1, 0.0);
        for (int i = 1; i < n; i++) {
            params[i] = params[i-1] + pdist(points[i-1], points[i]);
        }
        params[n] = params[n-1] + pdist(points[n-1], points[0]);

        // Domain
        auto [d0, d1] = curve.domain();
        std::cout << "Domain: [" << std::fixed << std::setprecision(4) << d0 << ", " << d1 << "]" << std::endl;

        double max_err = 0;
        for (int i = 0; i < n; i++) {
            // Map param to domain range
            double t = params[i];
            // Clamp to domain
            if (t < d0) t = d0;
            if (t > d1) t = d1;
            Point p = curve.point_at(t);
            double err = pdist(p, points[i]);
            if (err > max_err) max_err = err;
            std::cout << "  P[" << i << "] t=" << std::fixed << std::setprecision(4) << t
                      << "  eval=(" << std::setprecision(4) << p[0] << "," << p[1] << "," << p[2] << ")"
                      << "  expect=(" << points[i][0] << "," << points[i][1] << "," << points[i][2] << ")"
                      << "  err=" << std::scientific << std::setprecision(2) << err << std::endl;
        }
        std::cout << "Max interpolation error: " << std::scientific << std::setprecision(4)
                  << max_err << (max_err < 1e-6 ? " PASS" : " FAIL") << std::endl;

        // Verify closure: point at domain start ≈ point at domain end?
        Point pstart = curve.point_at(d0);
        Point pend = curve.point_at(d1);
        std::cout << "Start: (" << std::fixed << std::setprecision(4) << pstart[0] << "," << pstart[1] << "," << pstart[2] << ")" << std::endl;
        std::cout << "End:   (" << pend[0] << "," << pend[1] << "," << pend[2] << ")" << std::endl;

        session.add_curve(std::make_shared<NurbsCurve>(curve));
    }

    std::string filepath = (std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data" / "interpolated_curve.pb").string();
    session.pb_dump(filepath);
    std::cout << "\nSaved to: " << filepath << std::endl;

    return 0;
}
