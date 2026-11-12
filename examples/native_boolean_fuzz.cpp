#include "brep.h"
#include "brep_classify.h"
#include "xform.h"
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <string>

using namespace session_cpp;

// One random pair of primitives through all five Booleans:
//   native_boolean_fuzz <a 0-6> <b 0-6> <seed> [mode] [dump prefix]
// 0 box, 1 cylinder, 2 sphere, 3 cone, 4 pyramid, 5 torus, 6 block with hole.
// mode 0 turns and moves b, mode 1 only moves it: parallel axes, coplanar and tangent faces.
// A result must be a valid solid, its volume must add up around the common part, and points
// inside or outside both operands must lie where the operation puts them.
// Prints OK or one FAIL line per broken operation; the exit code is the number of failures.

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

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "usage: native_boolean_fuzz <a 0-6> <b 0-6> <seed> [mode] [dump prefix]\n";
        return 2;
    }
    const int ka = std::atoi(argv[1]), kb = std::atoi(argv[2]), seed = std::atoi(argv[3]);
    const int mode = argc > 4 ? std::atoi(argv[4]) : 0;
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
    if (argc > 5) {
        a.file_json_dump(std::string(argv[5]) + "_a.json");
        b.file_json_dump(std::string(argv[5]) + "_b.json");
    }

    const std::string label = std::to_string(ka) + " " + std::to_string(kb) + " " + std::to_string(seed) + " " + std::to_string(mode);
    const std::array<const char*, 5> names = {"intersection", "union", "a-b", "b-a", "xor"};
    const double va = a.volume_exact(), vb = b.volume_exact();
    std::array<BRep, 5> results;
    std::array<double, 5> volumes{};
    std::array<bool, 5> built{};
    int failures = 0;
    const auto fail = [&](int operation, const std::string& reason) {
        ++failures;
        std::cout << "FAIL " << label << " " << names[operation] << " : " << reason << "\n";
    };
    for (int i = 0; i < 5; ++i) {
        try {
            results[i] = i == 0 ? a.boolean_intersection(b) : i == 1 ? a.boolean_union(b) : i == 2 ? a.boolean_difference(b)
                : i == 3 ? b.boolean_difference(a) : a.boolean_xor(b);
            if (results[i].face_count() > 0 && (!results[i].is_valid() || !results[i].is_solid())) {
                fail(i, "result is not a valid solid");
                continue;
            }
            built[i] = true;
            // The exact integrator has a work budget some torus results exceed: then the probes below judge alone.
            try {
                volumes[i] = results[i].face_count() > 0 ? results[i].volume_exact() : 0.0;
            } catch (const std::exception&) {
                volumes[i] = std::nan("");
            }
        } catch (const std::exception& error) {
            fail(i, std::string(error.what()).substr(0, 160));
        }
    }
    if (built[0] && std::isfinite(volumes[0])) {
        const double vi = volumes[0];
        const std::array<double, 5> expected = {vi, va + vb - vi, va - vi, vb - vi, va + vb - 2 * vi};
        for (int i = 1; i < 5; ++i)
            if (built[i] && std::isfinite(volumes[i]) && std::abs(volumes[i] - expected[i]) > 1e-5 * (va + vb))
                fail(i, "volume " + std::to_string(volumes[i]) + " expected " + std::to_string(expected[i]));
    }
    using State = brep_classify::State;
    for (int i = 0; i < 5; ++i) {
        if (!built[i])
            continue;
        std::mt19937 probes(seed);
        int wrong = 0;
        for (int k = 0; k < 150; ++k) {
            std::array<double, 3> xyz;
            for (double& value : xyz) value = 3.2 * (probes() / 4294967296.0) - 1.6;
            const Point point(xyz[0], xyz[1], xyz[2]);
            const State in_a = brep_classify::point(a, point, 1e-7), in_b = brep_classify::point(b, point, 1e-7);
            if ((in_a != State::Inside && in_a != State::Outside) || (in_b != State::Inside && in_b != State::Outside))
                continue;
            const bool ia = in_a == State::Inside, ib = in_b == State::Inside;
            const bool expected = i == 0 ? ia && ib : i == 1 ? ia || ib : i == 2 ? ia && !ib : i == 3 ? ib && !ia : ia != ib;
            const State found = results[i].face_count() > 0 ? brep_classify::point(results[i], point, 1e-7) : State::Outside;
            if (found != (expected ? State::Inside : State::Outside))
                ++wrong;
        }
        if (wrong)
            fail(i, std::to_string(wrong) + " membership probes wrong");
    }
    if (!failures)
        std::cout << "OK " << label << " common volume " << volumes[0] << "\n";
    return failures;
}
