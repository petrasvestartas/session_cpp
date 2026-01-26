#include "src/nurbscurve.h"
#include "src/point.h"
#include "src/xform.h"
#include <iostream>
#include <iomanip>
#include <filesystem>

using namespace session_cpp;
namespace fs = std::filesystem;

int main() {
    // #include "nurbscurve.h"
    // #include "point.h"
    // #include "vector.h"

    std::vector<Point> points = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 1.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(3.0, 1.0, 0.0)
    };

    // The first the curve is closed or open
    // For linear curves use degree 1
    // When 3 points use degree 2 curve, Rhino default
    // When x>3 points use degree 3 curve
    NurbsCurve curve = NurbsCurve::create(false, 2, points);
    curve.set_domain(0.0, 1.0);

    // Minimal and Full String Representation
    std::string cstr = curve.str();
    std::string crepr = curve.repr();
    std::cout << "str: " << cstr << std::endl;
    std::cout << "repr: " << crepr << std::endl;

    // Copy (duplicates everything except guid)
    NurbsCurve ccopy = curve;
    NurbsCurve cother = NurbsCurve::create(false, 2, points);

    // Point division
    std::vector<Point> divided;
    curve.divide_by_count(10, divided);
    std::cout << "Divided points:" << std::endl;
    for (const auto& p : divided)
        std::cout << p << std::endl;

    // Serialization
    fs::path serial_dir = fs::path(__FILE__).parent_path() / "serialization";
    fs::create_directories(serial_dir);
    std::string json_path = (serial_dir / "test_nurbscurve.json").string();
    std::string bin_path = (serial_dir / "test_nurbscurve.bin").string();

    curve.json_dump(json_path);
    curve.protobuf_dump(bin_path);

    NurbsCurve loaded_json = NurbsCurve::json_load(json_path);
    NurbsCurve loaded_protobuf = NurbsCurve::protobuf_load(bin_path);

    std::cout << "Loaded JSON str: " << loaded_json.str() << std::endl;
    std::cout << "Loaded Protobuf str: " << loaded_protobuf.str() << std::endl;

    return 0;
}
