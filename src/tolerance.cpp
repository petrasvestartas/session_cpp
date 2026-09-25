#include "tolerance.h"
#include "point.h"
#include "tolerance.pb.h"
#include "vector.h"
#include <algorithm>
#include <cmath>
#include <fmt/core.h>
#include <fstream>
#include <stdexcept>

namespace session_cpp {

Tolerance TOLERANCE;

Tolerance::Tolerance(const std::string& unit) : _unit(unit) {}

void Tolerance::reset() {

    _absolute.reset();
    _relative.reset();
    _angular.reset();
    _approximation.reset();
    _precision.reset();
    _lineardeflection.reset();
    _angulardeflection.reset();
}

void Tolerance::set_unit(const std::string& value) {

    if (value != "M" && value != "MM")
        throw std::invalid_argument("Invalid unit: " + value);

    _unit = value;
}

void Tolerance::set_absolute(double value) {
    _absolute = value;
}

void Tolerance::set_relative(double value) {
    _relative = value;
}

void Tolerance::set_angular(double value) {
    _angular = value;
}

void Tolerance::set_approximation(double value) {
    _approximation = value;
}

void Tolerance::set_precision(int value) {

    if (value == 0)
        throw std::invalid_argument("Precision cannot be zero.");

    _precision = value;
}

void Tolerance::set_lineardeflection(double value) {
    _lineardeflection = value;
}

void Tolerance::set_angulardeflection(double value) {
    _angulardeflection = value;
}

ToleranceGuard Tolerance::temporary() {
    return ToleranceGuard(*this);
}

double Tolerance::tolerance(double truevalue, double rtol, double atol) const {
    return rtol * std::abs(truevalue) + atol;
}

bool Tolerance::compare(double a, double b, double rtol, double atol) const {
    return std::abs(a - b) <= tolerance(b, rtol, atol);
}

bool Tolerance::is_zero(double a) const {
    return std::abs(a) <= absolute();
}

bool Tolerance::is_positive(double a) const {
    return a > absolute();
}

bool Tolerance::is_negative(double a) const {
    return a < -absolute();
}

bool Tolerance::is_between(double value, double minval, double maxval) const {

    const double atol = absolute();

    return minval - atol <= value && value <= maxval + atol;
}

bool Tolerance::is_close(double a, double b) const {
    return compare(a, b, relative(), absolute());
}

bool Tolerance::is_angle_zero(double a) const {
    return std::abs(a) <= angular();
}

bool Tolerance::is_angles_close(double a, double b) const {
    return std::abs(a - b) <= angular();
}

bool Tolerance::is_point_close(const Point& a, const Point& b) const {

    const double dx = b[0] - a[0];
    const double dy = b[1] - a[1];
    const double dz = b[2] - a[2];

    return dx * dx + dy * dy + dz * dz <= absolute() * absolute();
}

bool Tolerance::is_vector_close(const Vector& a, const Vector& b) const {

    const double dx = b[0] - a[0];
    const double dy = b[1] - a[1];
    const double dz = b[2] - a[2];

    return dx * dx + dy * dy + dz * dz <= absolute() * absolute();
}

bool Tolerance::is_allclose(const std::vector<double>& a, const std::vector<double>& b) const {

    if (a.size() != b.size())
        return false;

    const double rtol = relative();
    const double atol = absolute();

    for (size_t i = 0; i < a.size(); ++i)
        if (!compare(a[i], b[i], rtol, atol))
            return false;

    return true;
}

std::string Tolerance::key(double x, double y, double z, int precision) const {

    const int prec = precision != -999 ? precision : this->precision();

    if (prec == 0)
        throw std::invalid_argument("Precision cannot be zero.");

    if (prec == -1)
        return fmt::format("{},{},{}", static_cast<int>(x), static_cast<int>(y), static_cast<int>(z));

    if (prec < -1) {
        const double factor = std::pow(10.0, -prec - 1);

        return fmt::format(
            "{},{},{}",
            static_cast<int>(std::round(x / factor) * factor),
            static_cast<int>(std::round(y / factor) * factor),
            static_cast<int>(std::round(z / factor) * factor)
        );
    }

    const double threshold = std::pow(10.0, -prec) * 0.5;

    if (std::abs(x) < threshold)
        x = 0.0;

    if (std::abs(y) < threshold)
        y = 0.0;

    if (std::abs(z) < threshold)
        z = 0.0;

    return fmt::format("{:.{}f},{:.{}f},{:.{}f}", x, prec, y, prec, z, prec);
}

std::string Tolerance::key_xy(double x, double y, int precision) const {

    const int prec = precision != -999 ? precision : this->precision();

    if (prec == 0)
        throw std::invalid_argument("Precision cannot be zero.");

    if (prec == -1)
        return fmt::format("{},{}", static_cast<int>(x), static_cast<int>(y));

    if (prec < -1) {
        const double factor = std::pow(10.0, -prec - 1);

        return fmt::format(
            "{},{}",
            static_cast<int>(std::round(x / factor) * factor),
            static_cast<int>(std::round(y / factor) * factor)
        );
    }

    const double threshold = std::pow(10.0, -prec) * 0.5;

    if (std::abs(x) < threshold)
        x = 0.0;

    if (std::abs(y) < threshold)
        y = 0.0;

    return fmt::format("{:.{}f},{:.{}f}", x, prec, y, prec);
}

std::string Tolerance::format_number(double number, int precision) const {

    const int prec = precision != -999 ? precision : this->precision();

    if (prec == 0)
        throw std::invalid_argument("Precision cannot be zero.");

    if (prec == -1)
        return fmt::format("{}", static_cast<int>(std::round(number)));

    if (prec < -1) {
        const double factor = std::pow(10.0, -prec - 1);

        return fmt::format("{}", static_cast<int>(std::round(number / factor) * factor));
    }

    return fmt::format("{:.{}f}", number, prec);
}

int Tolerance::precision_from_tolerance(double tol) const {

    const double value = tol >= 0 ? tol : absolute();

    if (value >= 1.0)
        return 0;

    const std::string text = fmt::format("{:e}", value);
    const size_t pos = text.find("e-");

    if (pos == std::string::npos)
        return 0;

    return std::stoi(text.substr(pos + 2));
}

double Tolerance::round_to(double value, int ndigits) {

    const double factor = std::pow(10.0, ndigits);

    return std::round(value * factor) / factor;
}

nlohmann::ordered_json Tolerance::jsondump() const {

    nlohmann::ordered_json data;
    data["absolute"] = absolute();
    data["angular"] = angular();
    data["angulardeflection"] = angulardeflection();
    data["approximation"] = approximation();
    data["lineardeflection"] = lineardeflection();
    data["precision"] = precision();
    data["relative"] = relative();
    data["type"] = "Tolerance";
    data["unit"] = unit();

    return data;
}

Tolerance Tolerance::jsonload(const nlohmann::json& data) {

    Tolerance tolerance(data["unit"]);
    tolerance.set_absolute(data["absolute"]);
    tolerance.set_angular(data["angular"]);
    tolerance.set_angulardeflection(data["angulardeflection"]);
    tolerance.set_approximation(data["approximation"]);
    tolerance.set_lineardeflection(data["lineardeflection"]);
    tolerance.set_precision(data["precision"]);
    tolerance.set_relative(data["relative"]);

    return tolerance;
}

std::string Tolerance::file_json_dumps() const {
    return jsondump().dump();
}

Tolerance Tolerance::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Tolerance::file_json_dump(const std::string& filename) const {

    std::ofstream file(filename);

    if (!file)
        throw std::runtime_error("Failed to open JSON file: " + filename);

    file << jsondump().dump(4);

    if (!file)
        throw std::runtime_error("Failed to write JSON file: " + filename);
}

Tolerance Tolerance::file_json_load(const std::string& filename) {

    std::ifstream file(filename);

    if (!file)
        throw std::runtime_error("Failed to open JSON file: " + filename);

    return jsonload(nlohmann::json::parse(file));
}

session_proto::Tolerance Tolerance::to_proto() const {

    session_proto::Tolerance proto;
    proto.set_unit(unit());
    proto.set_absolute(absolute());
    proto.set_relative(relative());
    proto.set_angular(angular());
    proto.set_approximation(approximation());
    proto.set_precision(precision());
    proto.set_lineardeflection(lineardeflection());
    proto.set_angulardeflection(angulardeflection());

    return proto;
}

Tolerance Tolerance::from_proto(const session_proto::Tolerance& proto) {

    Tolerance tolerance(proto.unit());
    tolerance.set_absolute(proto.absolute());
    tolerance.set_relative(proto.relative());
    tolerance.set_angular(proto.angular());
    tolerance.set_approximation(proto.approximation());
    tolerance.set_precision(proto.precision());
    tolerance.set_lineardeflection(proto.lineardeflection());
    tolerance.set_angulardeflection(proto.angulardeflection());

    return tolerance;
}

std::string Tolerance::pb_dumps() const {
    return to_proto().SerializeAsString();
}

Tolerance Tolerance::pb_loads(const std::string& data) {

    session_proto::Tolerance proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse Tolerance protobuf data");

    return from_proto(proto);
}

void Tolerance::pb_dump(const std::string& filename) const {

    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);

    if (!file)
        throw std::runtime_error("Failed to open protobuf file: " + filename);

    file.write(data.data(), data.size());

    if (!file)
        throw std::runtime_error("Failed to write protobuf file: " + filename);
}

Tolerance Tolerance::pb_load(const std::string& filename) {

    std::ifstream file(filename, std::ios::binary);

    if (!file)
        throw std::runtime_error("Failed to open protobuf file: " + filename);

    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    if (file.bad())
        throw std::runtime_error("Failed to read protobuf file: " + filename);

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
std::string Tolerance::str() const {
    return fmt::format("Tolerance({})", unit());
}

std::string Tolerance::repr() const {
    return fmt::format(
        "Tolerance(unit='{}', absolute={}, relative={}, angular={}, approximation={}, precision={}, lineardeflection={}, angulardeflection={})",
        unit(),
        absolute(),
        relative(),
        angular(),
        approximation(),
        precision(),
        lineardeflection(),
        angulardeflection()
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Utilities
// ═══════════════════════════════════════════════════════════════════════════
bool is_finite(double x) {
    return std::isfinite(x);
}

uint64_t unique_from_two_int(int a, int b) {

    const uint64_t lo = static_cast<uint64_t>(std::min(a, b));
    const uint64_t hi = static_cast<uint64_t>(std::max(a, b));

    return (hi << 32) | lo;
}

int wrap_index(int index, int n) {

    if (n == 0)
        return 0;

    return ((index % n) + n) % n;
}

double triangle_edge_by_angle(double edge_length, double angle_deg) {
    return edge_length * std::tan(Tolerance::to_radians(angle_deg));
}

double rad_to_deg(double radians) {
    return radians * Tolerance::TO_DEGREES;
}

double deg_to_rad(double degrees) {
    return degrees * Tolerance::TO_RADIANS;
}

int count_digits(double n) {

    const double value = std::abs(n);

    if (value < 1.0)
        return 0;

    return static_cast<int>(std::log10(value)) + 1;
}

} // namespace session_cpp
