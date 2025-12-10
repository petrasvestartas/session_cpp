#include "mini_test.h"
#include "vector.h"
#include "tolerance.h"

#include <cmath>
#include <filesystem>
#include <fstream>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Vector", "constructor") {
        // uncomment #include "vector.h"

        // Constructor
        Vector v(1.0, 2.0, 3.0);

        // Setters
        v[0] = 10.0;
        v[1] = 20.0;
        v[2] = 30.0;

        // Getters
        double x = v[0];
        double y = v[1];
        double z = v[2];

        // Minimal and full string representation
        std::string vstr = v.str();
        std::string vrepr = v.repr();

        // Copy (duplicate everything but guid)
        Vector vcopy = v.duplicate();
        Vector vother(1.0, 2.0, 3.0);

        // No-copy operators
        Vector vmult = v.duplicate();
        vmult *= 2.0;
        Vector vdiv = v.duplicate();
        vdiv /= 2.0;
        Vector vadd = v.duplicate();
        vadd += Vector(1.0, 1.0, 1.0);
        Vector vsub = v.duplicate();
        vsub -= Vector(1.0, 1.0, 1.0);

        // Copy operators
        Vector result_mul = v * 2.0;
        Vector result_div = v / 2.0;
        Vector result_add = v + Vector(1.0, 1.0, 1.0);
        Vector result_dif = v - Vector(1.0, 1.0, 1.0);

        // Static axis constructors
        Vector vx = Vector::x_axis();
        Vector vy = Vector::y_axis();
        Vector vz = Vector::z_axis();
        Vector vzero = Vector::zero();

        MINI_CHECK(v.name == "my_vector" && v[0] == 10.0 && v[1] == 20.0 && v[2] == 30.0 && v.guid != "");
        MINI_CHECK(x == 10.0 && y == 20.0 && z == 30.0);
        MINI_CHECK(vstr == "10.000000, 20.000000, 30.000000");
        MINI_CHECK(vrepr == "Vector(my_vector, 10.000000, 20.000000, 30.000000, 37.416574)");
        MINI_CHECK(vcopy == v && vcopy.guid != v.guid);
        MINI_CHECK(vother != v);
        MINI_CHECK(vmult[0] == 20.0 && vmult[1] == 40.0 && vmult[2] == 60.0);
        MINI_CHECK(vdiv[0] == 5.0 && vdiv[1] == 10.0 && vdiv[2] == 15.0);
        MINI_CHECK(vadd[0] == 11.0 && vadd[1] == 21.0 && vadd[2] == 31.0);
        MINI_CHECK(vsub[0] == 9.0 && vsub[1] == 19.0 && vsub[2] == 29.0);
        MINI_CHECK(result_mul[0] == 20.0 && result_mul[1] == 40.0 && result_mul[2] == 60.0);
        MINI_CHECK(result_div[0] == 5.0 && result_div[1] == 10.0 && result_div[2] == 15.0);
        MINI_CHECK(result_add[0] == 11.0 && result_add[1] == 21.0 && result_add[2] == 31.0);
        MINI_CHECK(result_dif[0] == 9.0 && result_dif[1] == 19.0 && result_dif[2] == 29.0);
        MINI_CHECK(vx[0] == 1.0 && vx[1] == 0.0 && vx[2] == 0.0);
        MINI_CHECK(vy[0] == 0.0 && vy[1] == 1.0 && vy[2] == 0.0);
        MINI_CHECK(vz[0] == 0.0 && vz[1] == 0.0 && vz[2] == 1.0);
        MINI_CHECK(vzero[0] == 0.0 && vzero[1] == 0.0 && vzero[2] == 0.0);
    }

    MINI_TEST("Vector", "arithmetic") {
        Vector v1(1.0, 2.0, 3.0);
        Vector v2(4.0, 5.0, 6.0);

        // Addition
        Vector sum = v1 + v2;
        MINI_CHECK(sum[0] == 5.0 && sum[1] == 7.0 && sum[2] == 9.0);

        // Subtraction
        Vector diff = v2 - v1;
        MINI_CHECK(diff[0] == 3.0 && diff[1] == 3.0 && diff[2] == 3.0);

        // Scalar multiplication
        Vector scaled = v1 * 2.0;
        MINI_CHECK(scaled[0] == 2.0 && scaled[1] == 4.0 && scaled[2] == 6.0);

        // Scalar division
        Vector divided = v2 / 2.0;
        MINI_CHECK(divided[0] == 2.0 && divided[1] == 2.5 && divided[2] == 3.0);
    }

    MINI_TEST("Vector", "magnitude") {
        Vector v(3.0, 4.0, 0.0);
        double len = v.magnitude();
        MINI_CHECK(Tolerance::round_to(len, Tolerance::ROUNDING) == 5.0);

        Vector unit(1.0, 0.0, 0.0);
        MINI_CHECK(unit.magnitude() == 1.0);
    }

    MINI_TEST("Vector", "normalize") {
        Vector v(3.0, 4.0, 0.0);
        Vector n = v; // copy
        n.normalize_self();

        MINI_CHECK(Tolerance::round_to(n.magnitude(), Tolerance::ROUNDING) == 1.0);
        MINI_CHECK(Tolerance::round_to(n[0], Tolerance::ROUNDING) == 0.6);
        MINI_CHECK(Tolerance::round_to(n[1], Tolerance::ROUNDING) == 0.8);
        MINI_CHECK(n[2] == 0.0);
    }

    MINI_TEST("Vector", "dot_product") {
        Vector v1(1.0, 0.0, 0.0);
        Vector v2(0.0, 1.0, 0.0);
        Vector v3(1.0, 0.0, 0.0);

        // Perpendicular vectors
        MINI_CHECK(v1.dot(v2) == 0.0);

        // Parallel vectors
        MINI_CHECK(v1.dot(v3) == 1.0);
    }

    MINI_TEST("Vector", "cross_product") {
        Vector v1(1.0, 0.0, 0.0);
        Vector v2(0.0, 1.0, 0.0);

        Vector cross = v1.cross(v2);
        MINI_CHECK(cross[0] == 0.0 && cross[1] == 0.0 && cross[2] == 1.0);
    }

    MINI_TEST("Vector", "json_roundtrip") {
        Vector v(42.1, 84.2, 126.3);
        v.name = "test_vector";

        std::string filename = "test_vector.json";

        // Write JSON to file
        std::ofstream ofs(filename);
        ofs << v.jsondump().dump(2);
        ofs.close();

        // Read JSON from file
        std::ifstream ifs(filename);
        nlohmann::json data = nlohmann::json::parse(ifs);
        ifs.close();
        Vector loaded = Vector::jsonload(data);

        MINI_CHECK(loaded.name == "test_vector");
        MINI_CHECK(Tolerance::round_to(loaded[0], Tolerance::ROUNDING) == Tolerance::round_to(42.1, Tolerance::ROUNDING));
        MINI_CHECK(Tolerance::round_to(loaded[1], Tolerance::ROUNDING) == Tolerance::round_to(84.2, Tolerance::ROUNDING));
        MINI_CHECK(Tolerance::round_to(loaded[2], Tolerance::ROUNDING) == Tolerance::round_to(126.3, Tolerance::ROUNDING));
    }

}
