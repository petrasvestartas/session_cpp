#include "tolerance.h"
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <fmt/core.h>

namespace session_cpp {

// Global tolerance instance
Tolerance TOLERANCE;

Tolerance::Tolerance(const std::string& unit) 
    : _unit(unit), _absolute(0), _relative(0), _angular(0), _approximation(0), 
      _precision(0), _lineardeflection(0), _angulardeflection(0),
      _has_absolute(false), _has_relative(false), _has_angular(false), 
      _has_approximation(false), _has_precision(false), 
      _has_lineardeflection(false), _has_angulardeflection(false) {
}

void Tolerance::reset() {
    _has_absolute = false;
    _has_relative = false;
    _has_angular = false;
    _has_approximation = false;
    _has_precision = false;
    _has_lineardeflection = false;
    _has_angulardeflection = false;
}

void Tolerance::set_unit(const std::string& value) {
    if (value != "M" && value != "MM") {
        throw std::invalid_argument("Invalid unit: " + value);
    }
    _unit = value;
}


void Tolerance::set_approximation(double value) {
    _approximation = value;
    _has_approximation = true;
}

void Tolerance::set_precision(int value) {
    if (value == 0) {
        throw std::invalid_argument("Precision cannot be zero.");
    }
    _precision = value;
    _has_precision = true;
}

void Tolerance::set_lineardeflection(double value) {
    _lineardeflection = value;
    _has_lineardeflection = true;
}

void Tolerance::set_angulardeflection(double value) {
    _angulardeflection = value;
    _has_angulardeflection = true;
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
    double atol = absolute();
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

std::string Tolerance::key(double x, double y, double z, int precision) const {
    int prec = (precision != -999) ? precision : this->precision();
    
    if (prec == 0) {
        throw std::invalid_argument("Precision cannot be zero.");
    }
    
    if (prec == -1) {
        return fmt::format("{},{},{}", static_cast<int>(x), static_cast<int>(y), static_cast<int>(z));
    }
    
    if (prec < -1) {
        int p = -prec - 1;
        double factor = std::pow(10.0, p);
        return fmt::format("{},{},{}",
               static_cast<int>(std::round(x / factor) * factor),
               static_cast<int>(std::round(y / factor) * factor),
               static_cast<int>(std::round(z / factor) * factor));
    }
    
    // Handle -0.0 case: if value rounds to zero at this precision, use positive zero
    double threshold = std::pow(10.0, -prec) * 0.5;
    if (std::abs(x) < threshold) x = 0.0;
    if (std::abs(y) < threshold) y = 0.0;
    if (std::abs(z) < threshold) z = 0.0;
    
    return fmt::format("{:.{}f},{:.{}f},{:.{}f}", x, prec, y, prec, z, prec);
}

std::string Tolerance::key_xy(double x, double y, int precision) const {
    int prec = (precision != -999) ? precision : this->precision();
    
    if (prec == 0) {
        throw std::invalid_argument("Precision cannot be zero.");
    }
    
    if (prec == -1) {
        return fmt::format("{},{}", static_cast<int>(x), static_cast<int>(y));
    }
    
    if (prec < -1) {
        int p = -prec - 1;
        double factor = std::pow(10.0, p);
        return fmt::format("{},{}",
               static_cast<int>(std::round(x / factor) * factor),
               static_cast<int>(std::round(y / factor) * factor));
    }
    
    // Handle -0.0 case: if value rounds to zero at this precision, use positive zero
    double threshold = std::pow(10.0, -prec) * 0.5;
    if (std::abs(x) < threshold) x = 0.0;
    if (std::abs(y) < threshold) y = 0.0;
    
    return fmt::format("{:.{}f},{:.{}f}", x, prec, y, prec);
}

std::string Tolerance::format_number(double number, int precision) const {
    int prec = (precision != -999) ? precision : this->precision();
    
    if (prec == 0) {
        throw std::invalid_argument("Precision cannot be zero.");
    }
    
    if (prec == -1) {
        return fmt::format("{}", static_cast<int>(std::round(number)));
    }
    
    if (prec < -1) {
        int p = -prec - 1;
        double factor = std::pow(10.0, p);
        return fmt::format("{}", static_cast<int>(std::round(number / factor) * factor));
    }
    
    return fmt::format("{:.{}f}", number, prec);
}

int Tolerance::precision_from_tolerance(double tol) const {
    double tolerance_val = (tol >= 0) ? tol : absolute();
    if (tolerance_val < 1.0) {
        std::ostringstream oss;
        oss << std::scientific << tolerance_val;
        std::string s = oss.str();
        size_t pos = s.find("e-");
        if (pos != std::string::npos) {
            return std::stoi(s.substr(pos + 2));
        }
    }
    return 0;
}

bool is_finite(double x) {
    return std::isfinite(x);
}

} // namespace session_cpp
