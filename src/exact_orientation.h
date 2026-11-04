#pragma once

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace session_cpp::exact_orientation {

static_assert(sizeof(double) == sizeof(std::uint64_t) && std::numeric_limits<double>::is_iec559
    && std::numeric_limits<double>::digits == 53);

struct Coordinate {
    std::uint64_t mantissa;
    int shift;
    bool negative;
};

inline Coordinate coordinate(double value) {
    const std::uint64_t bits = std::bit_cast<std::uint64_t>(value);
    const int exponent = static_cast<int>((bits >> 52) & 2047);
    if (exponent == 2047)
        throw std::invalid_argument("Exact orientation requires finite coordinates");
    const std::uint64_t fraction = bits & ((std::uint64_t(1) << 52) - 1);
    return {exponent ? fraction | (std::uint64_t(1) << 52) : fraction,
        exponent ? exponent - 1 : 0, (bits >> 63) != 0};
}

inline void add_word(std::array<std::uint64_t, 67>& value, std::size_t index, std::uint64_t word) {
    while (word) {
        if (index == value.size())
            throw std::overflow_error("Exact orientation integer capacity exceeded");
        const std::uint64_t previous = value[index];
        value[index] += word;
        word = value[index] < previous ? 1 : 0;
        ++index;
    }
}

inline void add_shifted(std::array<std::uint64_t, 67>& value, std::uint64_t word, int shift) {
    const std::size_t index = static_cast<std::size_t>(shift / 64);
    const int offset = shift % 64;
    add_word(value, index, word << offset);
    if (offset)
        add_word(value, index + 1, word >> (64 - offset));
}

inline void product(std::array<std::uint64_t, 67>& positive, std::array<std::uint64_t, 67>& negative,
    double a, double b, bool subtract) {
    const Coordinate first = coordinate(a), second = coordinate(b);
    std::array<std::uint64_t, 67>& result = (first.negative != second.negative) != subtract ? negative : positive;
    const std::uint64_t a0 = first.mantissa & 0xffffffff, a1 = first.mantissa >> 32;
    const std::uint64_t b0 = second.mantissa & 0xffffffff, b1 = second.mantissa >> 32;
    const int shift = first.shift + second.shift;
    add_shifted(result, a0 * b0, shift);
    add_shifted(result, a0 * b1, shift + 32);
    add_shifted(result, a1 * b0, shift + 32);
    add_shifted(result, a1 * b1, shift + 64);
}

inline int compare(const std::array<std::uint64_t, 67>& first, const std::array<std::uint64_t, 67>& second) {
    for (std::size_t i = first.size(); i-- > 0;)
        if (first[i] != second[i])
            return first[i] > second[i] ? 1 : -1;
    return 0;
}

inline std::array<std::uint64_t, 67> difference(const std::array<std::uint64_t, 67>& first, const std::array<std::uint64_t, 67>& second) {
    std::array<std::uint64_t, 67> result{};
    std::uint64_t borrow = 0;
    for (std::size_t i = 0; i < result.size(); ++i) {
        const std::uint64_t part = first[i] - second[i];
        result[i] = part - borrow;
        borrow = (first[i] < second[i]) || (part < borrow);
    }
    return result;
}

inline bool bit(const std::array<std::uint64_t, 67>& value, int index) {
    return ((value[static_cast<std::size_t>(index / 64)] >> (index % 64)) & 1) != 0;
}

inline bool below(const std::array<std::uint64_t, 67>& value, int count) {
    const std::size_t words = static_cast<std::size_t>(count / 64);
    for (std::size_t i = 0; i < words; ++i)
        if (value[i])
            return true;
    const int remainder = count % 64;
    return remainder && (value[words] & ((std::uint64_t(1) << remainder) - 1));
}

inline double magnitude(const std::array<std::uint64_t, 67>& value) {
    std::size_t high = value.size();
    while (high && value[high - 1] == 0)
        --high;
    if (!high)
        return 0;
    const int bits = static_cast<int>((high - 1) * 64) + 64 - std::countl_zero(value[high - 1]);
    const int shift = bits > 1127 ? bits - 53 : 1074;
    const std::size_t index = static_cast<std::size_t>(shift / 64);
    const int offset = shift % 64;
    std::uint64_t leading = value[index] >> offset;
    if (offset && index + 1 < value.size())
        leading |= value[index + 1] << (64 - offset);
    leading &= (std::uint64_t(1) << 53) - 1;
    if (shift && bit(value, shift - 1) && (below(value, shift - 1) || (leading & 1)))
        ++leading;
    const double result = std::ldexp(static_cast<double>(leading), shift - 2148);
    return result == 0 ? std::numeric_limits<double>::denorm_min() : result;
}

inline double exact(double ax, double ay, double bx, double by, double cx, double cy) {
    std::array<std::uint64_t, 67> positive{}, negative{};
    product(positive, negative, bx, cy, false);
    product(positive, negative, by, ax, false);
    product(positive, negative, ay, cx, false);
    product(positive, negative, bx, ay, true);
    product(positive, negative, ax, cy, true);
    product(positive, negative, by, cx, true);
    const int sign = compare(positive, negative);
    if (!sign)
        return 0;
    return sign > 0 ? magnitude(difference(positive, negative)) : -magnitude(difference(negative, positive));
}

inline double orient2d(double ax, double ay, double bx, double by, double cx, double cy) {
    const double left = (bx - ax) * (cy - ay), right = (by - ay) * (cx - ax);
    const double determinant = left - right;
    const double sum = std::abs(left) + std::abs(right);
    constexpr double epsilon = std::numeric_limits<double>::epsilon();
    if (std::isfinite(determinant) && std::isfinite(sum)
        && sum >= std::numeric_limits<double>::min() / epsilon
        && std::abs(determinant) > (8 * epsilon) * sum)
        return determinant;
    return exact(ax, ay, bx, by, cx, cy);
}

}
