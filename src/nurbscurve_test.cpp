#include "catch_amalgamated.hpp"
#include "nurbscurve.h"
#include "tolerance.h"
#include <cmath>

using namespace session_cpp;

// Test fixture for NurbsCurve tests
struct NurbsCurveFixture {
    NurbsCurve create_simple_curve() {
        // Create a simple cubic curve with 5 CVs
        NurbsCurve curve(3, false, 4, 5);
        curve.make_clamped_uniform_knot_vector(1.0);
        
        // Set CVs to create a simple curved line
        curve.set_cv(0, Point(0.0, 0.0, 0.0));
        curve.set_cv(1, Point(1.0, 1.0, 0.0));
        curve.set_cv(2, Point(2.0, 0.5, 0.0));
        curve.set_cv(3, Point(3.0, 1.5, 0.0));
        curve.set_cv(4, Point(4.0, 1.0, 0.0));
        
        return curve;
    }

    NurbsCurve create_rational_curve() {
        // Create a rational cubic curve
        NurbsCurve curve(3, true, 4, 5);
        curve.make_clamped_uniform_knot_vector(1.0);
        
        for (int i = 0; i < 5; i++) {
            curve.set_cv(i, Point(i * 1.0, std::sin(i * 0.5), 0.0));
            curve.set_weight(i, 1.0 + i * 0.1);
        }
        
        return curve;
    }
};

TEST_CASE_METHOD(NurbsCurveFixture, "NurbsCurve - Constructors", "[nurbscurve][constructors]") {
    SECTION("Default constructor") {
        NurbsCurve curve;
        REQUIRE(curve.dimension() == 0);
        REQUIRE(!curve.is_rational());
        REQUIRE(curve.order() == 0);
        REQUIRE(curve.cv_count() == 0);
    }
    
    SECTION("Parameterized constructor") {
        NurbsCurve curve(3, false, 4, 5);
        REQUIRE(curve.dimension() == 3);
        REQUIRE(!curve.is_rational());
        REQUIRE(curve.order() == 4);
        REQUIRE(curve.degree() == 3);
        REQUIRE(curve.cv_count() == 5);
        REQUIRE(curve.knot_count() == 7); // order + cv_count - 2
    }
    
    SECTION("Copy constructor") {
        auto original = create_simple_curve();
        NurbsCurve copy(original);
        
        REQUIRE(copy.dimension() == original.dimension());
        REQUIRE(copy.order() == original.order());
        REQUIRE(copy.cv_count() == original.cv_count());
        REQUIRE(copy.get_cv(2) == original.get_cv(2));
    }
}

TEST_CASE_METHOD(NurbsCurveFixture, "NurbsCurve - Accessors", "[nurbscurve][accessors]") {
    auto curve = create_simple_curve();
    
    SECTION("Basic properties") {
        REQUIRE(curve.dimension() == 3);
        REQUIRE(curve.order() == 4);
        REQUIRE(curve.degree() == 3);
        REQUIRE(curve.cv_count() == 5);
        REQUIRE(curve.cv_size() == 3); // non-rational, so dimension
        REQUIRE(!curve.is_rational());
    }
    
    SECTION("Domain") {
        auto [t0, t1] = curve.domain();
        REQUIRE(t0 == Catch::Approx(0.0));
        REQUIRE(t1 == Catch::Approx(2.0)); // (5-4+1) * 1.0
    }
    
    SECTION("Span count") {
        REQUIRE(curve.span_count() == 2); // cv_count - order + 1
    }
}

TEST_CASE_METHOD(NurbsCurveFixture, "NurbsCurve - Control Vertices", "[nurbscurve][cv]") {
    auto curve = create_simple_curve();
    
    SECTION("Get/Set CV") {
        Point pt = curve.get_cv(2);
        REQUIRE(pt[0] == Catch::Approx(2.0));
        REQUIRE(pt[1] == Catch::Approx(0.5));
        
        curve.set_cv(2, Point(2.5, 1.0, 0.5));
        Point new_pt = curve.get_cv(2);
        REQUIRE(new_pt[0] == Catch::Approx(2.5));
        REQUIRE(new_pt[1] == Catch::Approx(1.0));
        REQUIRE(new_pt[2] == Catch::Approx(0.5));
    }
    
    SECTION("Get/Set CV 4D") {
        double x, y, z, w;
        REQUIRE(curve.get_cv_4d(1, x, y, z, w));
        REQUIRE(x == Catch::Approx(1.0));
        REQUIRE(y == Catch::Approx(1.0));
        REQUIRE(w == Catch::Approx(1.0)); // non-rational, so weight is 1
    }
}

TEST_CASE_METHOD(NurbsCurveFixture, "NurbsCurve - Weights (Rational)", "[nurbscurve][weights]") {
    auto curve = create_rational_curve();
    
    SECTION("Get/Set weight") {
        double w = curve.weight(2);
        REQUIRE(w == Catch::Approx(1.2));
        
        curve.set_weight(2, 1.5);
        REQUIRE(curve.weight(2) == Catch::Approx(1.5));
    }
    
    SECTION("Make rational/non-rational") {
        NurbsCurve non_rat = create_simple_curve();
        REQUIRE(!non_rat.is_rational());
        
        non_rat.make_rational();
        REQUIRE(non_rat.is_rational());
        REQUIRE(non_rat.weight(0) == Catch::Approx(1.0));
    }
}

TEST_CASE_METHOD(NurbsCurveFixture, "NurbsCurve - Knot Operations", "[nurbscurve][knots]") {
    auto curve = create_simple_curve();
    
    SECTION("Knot access") {
        REQUIRE(curve.knot_count() == 7);
        
        // Clamped uniform should have repeated knots at ends
        // For cubic (degree 3), first 3 and last 3 knots should be equal
        REQUIRE(curve.knot(0) == Catch::Approx(curve.knot(1)));
        REQUIRE(curve.knot(1) == Catch::Approx(curve.knot(2)));
        REQUIRE(curve.knot(4) == Catch::Approx(curve.knot(5)));
        REQUIRE(curve.knot(5) == Catch::Approx(curve.knot(6)));
    }
    
    SECTION("Knot multiplicity") {
        int mult = curve.knot_multiplicity(0);
        REQUIRE(mult >= 1);
    }
    
    SECTION("Valid knot vector") {
        REQUIRE(curve.is_valid_knot_vector());
    }
    
    SECTION("Set domain") {
        REQUIRE(curve.set_domain(0.0, 10.0));
        auto [t0, t1] = curve.domain();
        REQUIRE(t0 == Catch::Approx(0.0));
        REQUIRE(t1 == Catch::Approx(10.0));
    }
}

TEST_CASE_METHOD(NurbsCurveFixture, "NurbsCurve - Evaluation", "[nurbscurve][evaluation]") {
    auto curve = create_simple_curve();
    
    SECTION("Point at parameter") {
        auto [t0, t1] = curve.domain();
        double t_mid = (t0 + t1) / 2.0;
        
        Point pt = curve.point_at(t_mid);
        REQUIRE(pt[0] >= 0.0);
        REQUIRE(pt[0] <= 4.0);
        REQUIRE(std::isfinite(pt[0]));
        REQUIRE(std::isfinite(pt[1]));
        REQUIRE(std::isfinite(pt[2]));
    }
    
    // Note: tangent_at() uses derivatives which need full implementation
    // Skipping for now as it requires complex derivative computation
    
    // Note: curvature_at() not implemented in current API
}

TEST_CASE_METHOD(NurbsCurveFixture, "NurbsCurve - Geometric Queries", "[nurbscurve][queries]") {
    auto curve = create_simple_curve();
    
    SECTION("Is closed") {
        REQUIRE(!curve.is_closed());
        
        // Make closed by setting first and last CV equal
        curve.set_cv(4, curve.get_cv(0));
        // Note: still not truly closed without periodic knots
    }
    
    SECTION("Is clamped") {
        REQUIRE(curve.is_clamped(2)); // both ends
        REQUIRE(curve.is_clamped(0)); // start
        REQUIRE(curve.is_clamped(1)); // end
    }
    
    SECTION("Bounding box") {
        auto bbox = curve.get_bounding_box();
        Point min_pt = bbox.min_point();
        Point max_pt = bbox.max_point();
        
        REQUIRE(min_pt[0] <= max_pt[0]);
        REQUIRE(min_pt[1] <= max_pt[1]);
        REQUIRE(min_pt[2] <= max_pt[2]);
    }
    
    SECTION("Arc length") {
        double len = curve.length();
        REQUIRE(len > 0.0);
        REQUIRE(std::isfinite(len));
    }
}

TEST_CASE_METHOD(NurbsCurveFixture, "NurbsCurve - Transformation", "[nurbscurve][transform]") {
    auto curve = create_simple_curve();
    Point original_pt = curve.get_cv(0);
    
    SECTION("Reverse") {
        Point first_before = curve.get_cv(0);
        Point last_before = curve.get_cv(curve.cv_count() - 1);
        
        REQUIRE(curve.reverse());
        
        Point first_after = curve.get_cv(0);
        Point last_after = curve.get_cv(curve.cv_count() - 1);
        
        REQUIRE(first_after == last_before);
        REQUIRE(last_after == first_before);
    }
    
    SECTION("Change dimension") {
        REQUIRE(curve.dimension() == 3);
        REQUIRE(curve.change_dimension(4));
        REQUIRE(curve.dimension() == 4);
    }
}

TEST_CASE_METHOD(NurbsCurveFixture, "NurbsCurve - Validation", "[nurbscurve][validation]") {
    SECTION("Valid curve") {
        auto curve = create_simple_curve();
        REQUIRE(curve.is_valid());
    }
    
    SECTION("Invalid curve") {
        NurbsCurve curve;
        REQUIRE(!curve.is_valid());
    }
}

TEST_CASE_METHOD(NurbsCurveFixture, "NurbsCurve - Serialization", "[nurbscurve][serialization]") {
    auto curve = create_simple_curve();
    
    SECTION("JSON dump") {
        auto json = curve.jsondump();
        
        REQUIRE(json.contains("dimension"));
        REQUIRE(json["dimension"] == 3);
        REQUIRE(json.contains("order"));
        REQUIRE(json["order"] == 4);
        REQUIRE(json.contains("cv_count"));
        REQUIRE(json["cv_count"] == 5);
    }
    
    SECTION("String representation") {
        std::string str = curve.to_string();
        REQUIRE(!str.empty());
        REQUIRE(str.find("NurbsCurve") != std::string::npos);
    }
}

TEST_CASE("NurbsCurve - Frames 3D (normal and Frenet)", "[nurbscurve][frames]") {
    // Build a clearly 3D curve (wavy helix)
    std::vector<Point> ctrl;
    for (int k = 0; k < 8; ++k) {
        double t = (static_cast<double>(k) / 7.0) * 2.0 * Tolerance::PI;
        double r = 1.5 + 0.3 * std::cos(3.0 * t);
        double x = r * std::cos(t);
        double y = r * std::sin(t);
        double z = 0.6 * t;
        ctrl.emplace_back(x, y, z);
    }

    NurbsCurve crv = NurbsCurve::create(false, 3, ctrl);
    auto [t0, t1] = crv.domain();
    double t = 0.5 * (t0 + t1);

    // Normal plane (plane normal = tangent)
    Vector T = crv.tangent_at(t); // already unit length
    REQUIRE(std::abs(T.magnitude() - 1.0) < 1e-6);
    Vector fallback = (std::abs(T[2]) < 0.9) ? Vector(0, 0, 1) : Vector(0, 1, 0);
    Vector e1 = T.cross(fallback);
    REQUIRE(e1.normalize_self());
    Vector e2 = T.cross(e1);
    REQUIRE(e2.normalize_self());
    REQUIRE(std::abs(e1.magnitude() - 1.0) < 1e-6);
    REQUIRE(std::abs(e2.magnitude() - 1.0) < 1e-6);
    REQUIRE(std::abs(e1.dot(T)) < 1e-6);
    REQUIRE(std::abs(e2.dot(T)) < 1e-6);
    REQUIRE(std::abs(e1.dot(e2)) < 1e-6);

    // Frenet frame (T, N, B)
    auto ders = crv.evaluate(t, 2);
    REQUIRE(ders.size() >= 3);
    Vector d1 = ders[1];
    Vector d2 = ders[2];
    Vector T_f = d1; REQUIRE(T_f.normalize_self());
    double proj = d2.dot(T_f);
    Vector N_raw = d2 - (T_f * proj);
    REQUIRE(N_raw.magnitude() > 1e-8);
    Vector N = N_raw; REQUIRE(N.normalize_self());
    Vector B = T_f.cross(N); REQUIRE(B.normalize_self());
    REQUIRE(std::abs(T_f.dot(N)) < 1e-6);
    REQUIRE(std::abs(T_f.dot(B)) < 1e-6);
    REQUIRE(std::abs(N.dot(B)) < 1e-6);
    Vector rhs = T_f.cross(N);
    REQUIRE(rhs.dot(B) > 0.999);
}
