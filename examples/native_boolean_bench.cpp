#include "brep.h"
#include "xform.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>

using namespace session_cpp;

// Fixed pairs of native_boolean_fuzz ("a b seed mode") through the five Booleans, one line each:
//   label op status bool_s faces edges vertices cv3d cv2d max_cv2d vol_s volume [message]
//   native_boolean_bench [label substring]

static BRep primitive(int kind, const std::array<double, 3>& s) {
    switch (kind) {
        case 0: return BRep::create_box(s[0], s[1], s[2]);
        case 1: return BRep::create_cylinder(0.5 * s[0], s[1]);
        case 2: return BRep::create_sphere(0.6 * s[0]);
        case 3: return BRep::create_cone(0.6 * s[0], s[1]);
        case 4: return BRep::create_pyramid(s[0], s[1]);
        case 5: return BRep::create_torus(0.8 * s[0], 0.25 * s[1]);
        default: return BRep::create_block_with_hole(1.2 * s[0], 1.2 * s[1], s[2], 0.3);
    }
}

static double seconds_since(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

int main(int argc, char** argv) {
    const std::array<std::array<int, 4>, 15> cases = {{
        {6, 5, 3, 0}, {6, 5, 3, 1}, {5, 0, 1, 0}, {5, 4, 1, 0}, {5, 0, 1, 1}, {5, 6, 2, 1}, {6, 2, 2, 0}, {5, 3, 2, 0},
        {5, 2, 1, 0}, {0, 0, 1, 0}, {0, 1, 1, 0}, {1, 2, 1, 0}, {3, 4, 1, 0}, {4, 2, 1, 0}, {2, 2, 1, 0},
    }};
    const std::array<const char*, 5> names = {"intersection", "union", "a-b", "b-a", "xor"};
    const std::string filter = argc > 1 ? argv[1] : "";
    std::string summaries;
    double grand_seconds = 0.0;
    int grand_max = 0, grand_budget = 0, grand_failed = 0, ran = 0;
    std::cout << std::setprecision(6) << "# label op status bool_s faces edges vertices cv3d cv2d max_cv2d vol_s volume\n" << std::flush;
    for (const auto& [ka, kb, seed, mode] : cases) {
        const std::string label = std::to_string(ka) + " " + std::to_string(kb) + " " + std::to_string(seed) + " " + std::to_string(mode);
        if (label.find(filter) == std::string::npos)
            continue;
        std::mt19937 engine(seed * 7919 + ka * 31 + kb);
        const auto unit = [&engine] { return engine() / 4294967296.0; };
        const auto size = [&unit] { return 0.6 + unit(); };
        const auto signed_unit = [&unit] { return 2.0 * unit() - 1.0; };
        std::array<double, 3> sa, sb, shift, axis;
        for (double& value : sa) value = size();
        for (double& value : sb) value = size();
        for (double& value : shift) value = 0.45 * signed_unit();
        for (double& value : axis) value = signed_unit();
        const double angle = 3.0 * signed_unit();
        const BRep a = primitive(ka, sa);
        BRep b = primitive(kb, sb);
        Xform placed = Xform::translation(shift[0], shift[1], shift[2]);
        if (mode == 0)
            placed = placed * Xform::rotation(Vector(axis[0], axis[1], axis[2]), angle);
        b.transform(placed);

        double case_seconds = 0.0;
        int case_max = 0, case_budget = 0, case_failed = 0;
        for (int i = 0; i < 5; ++i) {
            std::cout << "\"" << label << "\" " << names[i] << " ";
            BRep result;
            const auto start = std::chrono::steady_clock::now();
            try {
                result = i == 0 ? a.boolean_intersection(b) : i == 1 ? a.boolean_union(b) : i == 2 ? a.boolean_difference(b)
                    : i == 3 ? b.boolean_difference(a) : a.boolean_xor(b);
            } catch (const std::exception& error) {
                const double seconds = seconds_since(start);
                case_seconds += seconds;
                ++case_failed;
                std::cout << "FAIL " << seconds << " 0 0 0 0 0 0 - - " << std::string(error.what()).substr(0, 80) << "\n" << std::flush;
                continue;
            }
            const double seconds = seconds_since(start);
            case_seconds += seconds;
            int cv3d = 0, cv2d = 0, max2d = 0;
            for (const NurbsCurve& curve : result.m_curves_3d) cv3d += curve.cv_count();
            for (const NurbsCurve& curve : result.m_curves_2d) {
                cv2d += curve.cv_count();
                max2d = std::max(max2d, curve.cv_count());
            }
            case_max = std::max(case_max, max2d);
            std::cout << "ok " << seconds << " " << result.face_count() << " " << result.edge_count() << " " << result.vertex_count() << " "
                      << cv3d << " " << cv2d << " " << max2d << " " << std::flush;
            const auto volume_start = std::chrono::steady_clock::now();
            try {
                const double volume = result.face_count() > 0 ? result.volume_exact() : 0.0;
                std::cout << seconds_since(volume_start) << " " << volume << "\n" << std::flush;
            } catch (const std::exception&) {
                ++case_budget;
                std::cout << seconds_since(volume_start) << " budget\n" << std::flush;
            }
        }
        summaries += "SUMMARY \"" + label + "\" bool_s " + std::to_string(case_seconds) + " max_cv2d " + std::to_string(case_max)
            + " budget " + std::to_string(case_budget) + " failed " + std::to_string(case_failed) + "\n";
        grand_seconds += case_seconds;
        grand_max = std::max(grand_max, case_max);
        grand_budget += case_budget;
        grand_failed += case_failed;
        ++ran;
    }
    std::cout << summaries << "TOTAL cases " << ran << " bool_s " << grand_seconds << " max_cv2d " << grand_max << " budget " << grand_budget
              << " failed " << grand_failed << "\n" << std::flush;
    return 0;
}
