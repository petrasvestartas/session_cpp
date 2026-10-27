#include "nurbs_offset.h"
#include "nurbs_surface_span.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace session_cpp::nurbs_offset {
namespace {

constexpr double Epsilon = std::numeric_limits<double>::epsilon();

double vector_length(const Vector& vector) {
    return std::hypot(vector[0], vector[1], vector[2]);
}

void validate_options(double distance, double tolerance) {
    if (!std::isfinite(distance) || !std::isfinite(tolerance) || tolerance <= 0.0)
        throw std::invalid_argument("Offset requires finite distance and positive finite tolerance");
    if (tolerance < 128.0 * Epsilon * std::abs(distance))
        throw std::runtime_error("Offset tolerance is below displacement resolution");
}

void validate_controls(const std::vector<double>& controls) {
    for (double coordinate : controls) {
        if (!std::isfinite(coordinate))
            throw std::invalid_argument("Offset requires finite control points");
    }
}

void validate_resolution(double scale, double tolerance) {
    if (tolerance < 128.0 * Epsilon * scale)
        throw std::runtime_error("Offset tolerance is below coordinate resolution");
}

void curve_control(NurbsCurve& curve, int index, const Point& point) {
    const double weight = curve.weight(index);
    double* coordinates = curve.cv(index);
    for (int d = 0; d < 3; ++d)
        coordinates[d] = point[d] * weight;
}

void surface_control(
    NurbsSurface& surface,
    int i,
    int j,
    const Point& point
) {
    const double weight = surface.weight(i, j);
    double* coordinates = surface.cv(i, j);
    for (int d = 0; d < 3; ++d)
        coordinates[d] = point[d] * weight;
}

void normalize_weights(std::vector<double>& coordinates, double largest) {
    if (largest == 1.0)
        return;
    for (double& coordinate : coordinates) {
        coordinate /= largest;
        if (!std::isfinite(coordinate))
            throw std::runtime_error("Offset rational normalization exceeded coordinate range");
    }
}

void validate_curve(NurbsCurve& source, double tolerance) {
    if (!source.is_valid() || source.dimension() != 3)
        throw std::invalid_argument("Offset requires a valid three-dimensional NURBS curve");
    double largest = 0.0;
    for (int i = 0; i < source.cv_count(); ++i) {
        if (!(source.weight(i) > 0.0) || !std::isfinite(source.weight(i)))
            throw std::invalid_argument("Offset requires positive finite rational weights");
        largest = std::max(largest, source.weight(i));
    }
    normalize_weights(source.m_cv, largest);
    double scale = 0.0;
    for (int i = 0; i < source.cv_count(); ++i) {
        if (!(source.weight(i) > 0.0) || !std::isfinite(source.weight(i)))
            throw std::invalid_argument("Offset requires positive finite rational weights");
        const Point point = source.get_cv(i);
        for (int d = 0; d < 3; ++d)
            scale = std::max(scale, std::abs(point[d]));
    }
    validate_controls(source.m_cv);
    validate_controls(source.m_nurbsknot);
    validate_resolution(scale, tolerance);
}

void validate_surface(NurbsSurface& source, double tolerance) {
    if (!source.is_valid() || source.dimension() != 3)
        throw std::invalid_argument("Offset requires a valid three-dimensional NURBS surface");
    double largest = 0.0;
    for (int i = 0; i < source.cv_count(0); ++i)
        for (int j = 0; j < source.cv_count(1); ++j) {
            if (!(source.weight(i, j) > 0.0) || !std::isfinite(source.weight(i, j)))
                throw std::invalid_argument("Offset requires positive finite rational weights");
            largest = std::max(largest, source.weight(i, j));
        }
    normalize_weights(source.m_cv, largest);
    double scale = 0.0;
    for (int i = 0; i < source.cv_count(0); ++i)
        for (int j = 0; j < source.cv_count(1); ++j) {
            if (!(source.weight(i, j) > 0.0) || !std::isfinite(source.weight(i, j)))
                throw std::invalid_argument("Offset requires positive finite rational weights");
            const Point point = source.get_cv(i, j);
            for (int d = 0; d < 3; ++d)
                scale = std::max(scale, std::abs(point[d]));
        }
    validate_controls(source.m_cv);
    validate_controls(source.m_nurbsknot[0]);
    validate_controls(source.m_nurbsknot[1]);
    validate_resolution(scale, tolerance);
}

Vector regular_normal(const NurbsSurface& source, double u, double v) {
    const std::vector<Vector> derivatives = source.evaluate(u, v, 1);
    Vector su = derivatives[2];
    Vector sv = derivatives[1];
    const double lu = vector_length(su);
    const double lv = vector_length(sv);
    if (!std::isfinite(lu) || !std::isfinite(lv) || lu <= 0.0 || lv <= 0.0)
        throw std::runtime_error("Surface offset has an undefined one-sided normal");
    const Vector normal = (su / lu).cross(sv / lv);
    const double magnitude = vector_length(normal);
    if (!std::isfinite(magnitude) || magnitude <= 128.0 * Epsilon)
        throw std::runtime_error("Surface offset has an undefined one-sided normal");
    return normal / magnitude;
}

void check_surface_continuity(const NurbsSurface& source) {
    const double order = std::max(source.order(0), source.order(1));
    const double angular = 4096.0 * Epsilon * order * order;
    for (int direction = 0; direction < 2; ++direction) {
        const std::vector<double> spans = source.get_span_vector(direction);
        const std::vector<double> transverse = source.get_span_vector(1 - direction);
        const std::vector<double>& knots = source.m_nurbsknot[direction];
        for (size_t i = 1; i + 1 < spans.size(); ++i) {
            if (std::count(knots.begin(), knots.end(), spans[i]) < source.degree(direction))
                continue;
            const std::pair<NurbsSurface, NurbsSurface> pieces = source.split(direction, spans[i]);
            if (!pieces.first.is_valid() || !pieces.second.is_valid())
                throw std::runtime_error("Surface offset could not inspect one-sided continuity");
            const int samples = std::max(7, 2 * source.degree(1 - direction) + 3);
            for (size_t j = 1; j < transverse.size(); ++j)
                for (int k = 1; k <= samples; ++k) {
                    const double varying = transverse[j - 1] + (transverse[j] - transverse[j - 1]) * k / (samples + 1);
                    const double u = direction == 0 ? spans[i] : varying;
                    const double v = direction == 0 ? varying : spans[i];
                    const Vector first = regular_normal(pieces.first, u, v);
                    const Vector second = regular_normal(pieces.second, u, v);
                    if ((first - second).magnitude() > angular)
                        throw std::runtime_error("Surface offset requires splitting at discontinuous one-sided normals");
                }
        }
    }
}

std::array<double, 4> radial_control(
    const Point& point,
    double weight,
    const Point& center,
    double radius
) {
    const Vector delta = (point - center) / radius;
    return {delta[0] * weight, delta[1] * weight, delta[2] * weight, weight};
}

bool unit_sphere_identity(const std::vector<std::array<double, 4>>& poles, int nu, int nv) {
    const int choose[5][5] = {{1}, {1, 1}, {1, 2, 1}, {1, 3, 3, 1}, {1, 4, 6, 4, 1}};
    const int pu = nu - 1;
    const int pv = nv - 1;
    if (pu > 2 || pv > 2)
        return false;
    double weight = 0.0;
    for (const std::array<double, 4>& pole : poles)
        weight = std::max(weight, std::abs(pole[3]));
    for (int k = 0; k <= 2 * pu; ++k)
        for (int l = 0; l <= 2 * pv; ++l) {
            double coefficient = 0.0;
            double magnitude = 0.0;
            for (int i = std::max(0, k - pu); i <= std::min(pu, k); ++i)
                for (int j = std::max(0, l - pv); j <= std::min(pv, l); ++j) {
                    const std::array<double, 4>& a = poles[i * nv + j];
                    const std::array<double, 4>& b = poles[(k - i) * nv + l - j];
                    const double factor = double(choose[pu][i] * choose[pu][k - i]) / choose[2 * pu][k]
                        * double(choose[pv][j] * choose[pv][l - j]) / choose[2 * pv][l];
                    for (int d = 0; d < 4; ++d) {
                        const double term = factor * (a[d] / weight) * (b[d] / weight);
                        coefficient += d == 3 ? -term : term;
                        magnitude += std::abs(term);
                    }
                }
            if (!std::isfinite(coefficient) || std::abs(coefficient) > 512.0 * Epsilon * magnitude)
                return false;
        }
    return true;
}

bool circle_center(const NurbsCurve& source, Point& center, Vector& axis) {
    const double a = source.domain_start();
    const double h = source.domain_end() - a;
    const Point p = source.point_at(a + h * 0.07);
    const Vector u = source.point_at(a + h * 0.38) - p;
    const Vector v = source.point_at(a + h * 0.73) - p;
    axis = u.cross(v);
    const double square = axis.dot(axis);
    if (square <= 1e-20 * u.dot(u) * v.dot(v))
        return false;
    center = p + (v.cross(axis) * u.dot(u) + axis.cross(u) * v.dot(v)) / (2.0 * square);
    axis = axis / std::sqrt(square);
    return true;
}

bool circle_identity(
    const NurbsCurve& source,
    const Point& center,
    const Vector& axis,
    double radius
) {
    const std::vector<double> spans = source.get_span_vector();
    for (size_t i = 0; i + 1 < spans.size(); ++i) {
        NurbsCurve span = source;
        if (!span.trim(spans[i], spans[i + 1]) || span.cv_count() != 3)
            return false;
        std::vector<std::array<double, 4>> poles;
        for (int j = 0; j < 3; ++j) {
            const Point point = span.get_cv(j);
            if (std::abs((point - center).dot(axis)) > radius * 512.0 * Epsilon)
                return false;
            poles.push_back(radial_control(point, span.weight(j), center, radius));
        }
        if (!unit_sphere_identity(poles, 3, 1))
            return false;
    }
    return true;
}

double circle_orientation(const NurbsCurve& source, const Point& center, const Vector& normal) {
    const std::vector<double> spans = source.get_span_vector();
    double orientation = 0.0;
    for (size_t i = 0; i + 1 < spans.size(); ++i) {
        const double parameter = (spans[i] + spans[i + 1]) * 0.5;
        const std::vector<Vector> derivatives = source.evaluate(parameter, 1);
        const Vector side = derivatives[1].cross(normal);
        if (side.magnitude() <= 128.0 * Epsilon * derivatives[1].magnitude())
            throw std::runtime_error("Circle offset has a singular parameter span");
        const Vector radial = source.point_at(parameter) - center;
        const double sign = side.dot(radial) > 0.0 ? 1.0 : -1.0;
        if (orientation != 0.0 && sign != orientation)
            throw std::runtime_error("Circle offset changes direction across a knot");
        orientation = sign;
    }
    return orientation;
}

bool exact_curve(
    const NurbsCurve& source,
    double distance,
    const Vector& normal,
    NurbsCurve& result
) {
    if (source.degree() == 1 && source.cv_count() == 2) {
        const Vector tangent = source.get_cv(1) - source.get_cv(0);
        const Vector side = tangent.cross(normal);
        if (side.magnitude() <= 128.0 * Epsilon * tangent.magnitude())
            throw std::runtime_error("Curve offset direction is singular");
        result = source;
        for (int i = 0; i < 2; ++i)
            curve_control(result, i, source.get_cv(i) + side.normalized() * distance);
        return true;
    }
    Point center;
    Vector axis;
    if (source.degree() != 2 || !circle_center(source, center, axis))
        return false;
    if (axis.cross(normal).magnitude() > 128.0 * Epsilon)
        return false;
    const Point point = source.point_at_middle();
    const double radius = point.distance(center);
    if (!circle_identity(source, center, axis, radius))
        return false;
    const double sign = circle_orientation(source, center, normal);
    const double factor = 1.0 + sign * distance / radius;
    if (std::abs(factor) <= 128.0 * Epsilon)
        throw std::runtime_error("Curve offset collapses to a point");
    result = source;
    for (int i = 0; i < source.cv_count(); ++i)
        curve_control(result, i, center + (source.get_cv(i) - center) * factor);
    return true;
}

struct CurveValue {
    Point point;
    Vector derivative;
};

struct CurveField {
    const NurbsCurve& source;
    double distance;
    Vector normal;

    CurveValue at(double parameter) const {
        const std::vector<Vector> derivatives = source.evaluate(parameter, 2);
        const Vector side = derivatives[1].cross(normal);
        const double length = side.magnitude();
        if (!std::isfinite(length) || length <= 128.0 * Epsilon * derivatives[1].magnitude())
            throw std::runtime_error("Curve offset has an undefined tangent or normal");
        const Vector unit = side / length;
        const Vector change = derivatives[2].cross(normal);
        const Vector derivative = derivatives[1] + (change - unit * unit.dot(change)) * (distance / length);
        if (!std::isfinite(derivative.magnitude()) || derivative.magnitude() <= 128.0 * Epsilon * derivatives[1].magnitude())
            throw std::runtime_error("Curve offset has a cusp or collapse");
        const Vector position = derivatives[0] + unit * distance;
        for (int d = 0; d < 3; ++d)
            if (!std::isfinite(position[d]))
                throw std::runtime_error("Curve offset evaluation overflowed");
        return {{position[0], position[1], position[2]}, derivative};
    }
};

std::array<Point, 4> cubic_controls(const CurveValue& a, const CurveValue& b, double interval) {
    return {a.point, a.point + a.derivative * (interval / 3.0), b.point - b.derivative * (interval / 3.0), b.point};
}

Point cubic_point(const std::array<Point, 4>& controls, double t) {
    Point point;
    const double s = 1.0 - t;
    const double weights[4] = {s * s * s, 3.0 * s * s * t, 3.0 * s * t * t, t * t * t};
    for (int d = 0; d < 3; ++d)
        for (int i = 0; i < 4; ++i)
            point[d] += controls[i][d] * weights[i];
    return point;
}

struct CurvePiece {
    double a;
    double b;
    int depth;
};

void append_curve_piece(
    const CurveField& field,
    const CurvePiece& piece,
    double tolerance,
    std::vector<Point>& controls,
    std::vector<double>& knots
) {
    if (controls.size() > 65536)
        throw std::runtime_error("Curve offset exceeded its control-point budget");
    const CurveValue a = field.at(piece.a);
    const CurveValue b = field.at(piece.b);
    const std::array<Point, 4> poles = cubic_controls(a, b, piece.b - piece.a);
    double error = 0.0;
    for (int i = 1; i < 8; ++i) {
        const double t = i / 8.0;
        const CurveValue value = field.at(piece.a + (piece.b - piece.a) * t);
        error = std::max(error, value.point.distance(cubic_point(poles, t)));
    }
    if (error > tolerance * 0.25) {
        const double middle = (piece.a + piece.b) * 0.5;
        if (piece.depth >= 24 || controls.size() > 65536 || middle <= piece.a || middle >= piece.b)
            throw std::runtime_error("Curve offset refinement cannot meet sampled tolerance");
        append_curve_piece(field, {piece.a, middle, piece.depth + 1}, tolerance, controls, knots);
        append_curve_piece(field, {middle, piece.b, piece.depth + 1}, tolerance, controls, knots);
        return;
    }
    if (controls.empty()) {
        controls.push_back(poles[0]);
        knots.insert(knots.end(), 3, piece.a);
    } else if (controls.back().distance(poles[0]) > tolerance * 0.25) {
        throw std::runtime_error("Curve offset is discontinuous at a knot");
    }
    for (int i = 1; i < 4; ++i)
        controls.push_back(poles[i]);
    knots.insert(knots.end(), 3, piece.b);
}

bool sphere_center(const NurbsSurface& source, Point& center) {
    const std::pair<double, double> u = source.domain(0);
    const std::pair<double, double> v = source.domain(1);
    const double du = u.second - u.first;
    const double dv = v.second - v.first;
    const Point p = source.point_at(u.first + du * 0.03, v.first + dv * 0.42);
    const Vector a = source.point_at(u.first + du * 0.29, v.first + dv * 0.23) - p;
    const Vector b = source.point_at(u.first + du * 0.57, v.first + dv * 0.52) - p;
    const Vector c = source.point_at(u.first + du * 0.81, v.first + dv * 0.79) - p;
    const double determinant = a.dot(b.cross(c));
    if (std::abs(determinant) <= 1e-10 * a.magnitude() * b.magnitude() * c.magnitude())
        return false;
    center = p + (b.cross(c) * a.dot(a) + c.cross(a) * b.dot(b) + a.cross(b) * c.dot(c)) / (2.0 * determinant);
    return true;
}

double sphere_orientation(const NurbsSurface& source, const Point& center) {
    const std::vector<double> u = source.get_span_vector(0);
    const std::vector<double> v = source.get_span_vector(1);
    double orientation = 0.0;
    for (size_t i = 0; i + 1 < u.size(); ++i)
        for (size_t j = 0; j + 1 < v.size(); ++j) {
            const double x = (u[i] + u[i + 1]) * 0.5;
            const double y = (v[j] + v[j + 1]) * 0.5;
            const std::vector<Vector> derivatives = source.evaluate(x, y, 1);
            const Vector area = derivatives[2].cross(derivatives[1]);
            if (area.magnitude() <= 128.0 * Epsilon * derivatives[1].magnitude() * derivatives[2].magnitude())
                throw std::runtime_error("Sphere offset has a singular parameter patch");
            const double sign = area.dot(source.point_at(x, y) - center) > 0.0 ? 1.0 : -1.0;
            if (orientation != 0.0 && sign != orientation)
                throw std::runtime_error("Sphere offset changes orientation across a knot");
            orientation = sign;
        }
    return orientation;
}

bool cylinder_profile(const NurbsSurface& source, Point& center, Vector& axis) {
    if (source.degree(0) != 2 || source.degree(1) != 1 || source.cv_count(1) != 2)
        return false;
    const NurbsCurve profile = source.iso_curve(0, source.domain(1).first);
    if (!circle_center(profile, center, axis))
        return false;
    const double radius = profile.point_at_middle().distance(center);
    if (!circle_identity(profile, center, axis, radius))
        return false;
    circle_orientation(profile, center, axis);
    const Vector translation = source.get_cv(0, 1) - source.get_cv(0, 0);
    const double length = translation.magnitude();
    if (length == 0.0 || translation.cross(axis).magnitude() > 128.0 * Epsilon * length)
        return false;
    const double ratio = source.weight(0, 1) / source.weight(0, 0);
    for (int i = 0; i < source.cv_count(0); ++i) {
        const Vector delta = source.get_cv(i, 1) - source.get_cv(i, 0);
        if ((delta - translation).magnitude() > 128.0 * Epsilon * length)
            return false;
        const double weight = source.weight(i, 1) / source.weight(i, 0);
        if (std::abs(weight - ratio) > 128.0 * Epsilon * ratio)
            return false;
    }
    return true;
}

bool exact_cylinder(const NurbsSurface& source, double distance, NurbsSurface& result) {
    Point center;
    Vector axis;
    if (!cylinder_profile(source, center, axis))
        return false;
    const std::pair<double, double> u = source.domain(0);
    const std::pair<double, double> v = source.domain(1);
    const double um = (u.first + u.second) * 0.5;
    const double vm = (v.first + v.second) * 0.5;
    const Vector delta = source.point_at(um, vm) - center;
    const Vector radial = delta - axis * axis.dot(delta);
    const double radius = radial.magnitude();
    const double sign = source.normal_at(um, vm).dot(radial) > 0.0 ? 1.0 : -1.0;
    const double factor = 1.0 + sign * distance / radius;
    if (std::abs(factor) <= 128.0 * Epsilon)
        throw std::runtime_error("Cylinder offset collapses to its axis");
    result = source;
    result.m_mesh.clear();
    for (int i = 0; i < source.cv_count(0); ++i)
        for (int j = 0; j < 2; ++j) {
            const Point point = source.get_cv(i, j);
            const Vector displacement = point - center;
            const Vector r = displacement - axis * axis.dot(displacement);
            surface_control(
                result,
                i,
                j,
                point + r * (factor - 1.0)
            );
        }
    return true;
}

bool sphere_identity(const NurbsSurface& source, const Point& center, double radius) {
    const std::vector<double> u = source.get_span_vector(0);
    const std::vector<double> v = source.get_span_vector(1);
    for (size_t i = 0; i + 1 < u.size(); ++i)
        for (size_t j = 0; j + 1 < v.size(); ++j) {
            NurbsSurface patch = source;
            if (!patch.trim(0, {u[i], u[i + 1]}) || !patch.trim(1, {v[j], v[j + 1]}))
                return false;
            if (patch.cv_count(0) != 3 || patch.cv_count(1) != 3)
                return false;
            std::vector<std::array<double, 4>> poles;
            for (int a = 0; a < 3; ++a)
                for (int b = 0; b < 3; ++b)
                    poles.push_back(radial_control(patch.get_cv(a, b), patch.weight(a, b), center, radius));
            if (!unit_sphere_identity(poles, 3, 3))
                return false;
        }
    return true;
}

bool planar_translation(
    const NurbsSurface& source,
    double distance,
    double tolerance,
    NurbsSurface& result
) {
    if (source.degree(0) != 1 || source.degree(1) != 1 || source.is_rational())
        return false;
    Plane plane;
    if (!source.is_planar(&plane, 1e-12))
        return false;
    const std::vector<double> u = source.get_span_vector(0);
    const std::vector<double> v = source.get_span_vector(1);
    const Vector normal = regular_normal(source, (u[0] + u[1]) * 0.5, (v[0] + v[1]) * 0.5);
    for (size_t i = 1; i < u.size(); ++i)
        for (size_t j = 1; j < v.size(); ++j) {
            NurbsSurface patch = source;
            if (!patch.trim(0, {u[i - 1], u[i]}) || !patch.trim(1, {v[j - 1], v[j]}))
                throw std::runtime_error("Surface offset could not inspect a planar knot span");
            for (int a = 0; a < 2; ++a)
                for (int b = 0; b < 2; ++b) {
                    const std::vector<Vector> derivatives = patch.evaluate(a ? u[i] : u[i - 1], b ? v[j] : v[j - 1], 1);
                    const double lu = vector_length(derivatives[2]);
                    const double lv = vector_length(derivatives[1]);
                    if (!std::isfinite(lu) || !std::isfinite(lv) || lu <= 0.0 || lv <= 0.0)
                        throw std::runtime_error("Surface offset has a singular planar parametrization");
                    const Vector area = (derivatives[2] / lu).cross(derivatives[1] / lv);
                    const double projection = area.dot(normal);
                    if (projection <= 128.0 * Epsilon)
                        throw std::runtime_error("Surface offset has a singular planar parametrization");
                    if (std::abs(distance) * vector_length(area - normal * projection) > tolerance * 0.125 * projection)
                        return false;
                }
        }
    result = source;
    result.m_mesh.clear();
    for (int i = 0; i < source.cv_count(0); ++i)
        for (int j = 0; j < source.cv_count(1); ++j)
            surface_control(result, i, j, source.get_cv(i, j) + normal * distance);
    return true;
}

bool exact_surface(
    const NurbsSurface& source,
    double distance,
    double tolerance,
    NurbsSurface& result
) {
    const std::pair<double, double> u = source.domain(0);
    const std::pair<double, double> v = source.domain(1);
    const double um = (u.first + u.second) * 0.5;
    const double vm = (v.first + v.second) * 0.5;
    if (planar_translation(source, distance, tolerance, result))
        return true;
    Point center;
    if (source.degree(0) != 2 || source.degree(1) != 2 || !sphere_center(source, center))
        return false;
    const Point point = source.point_at(um, vm);
    const double radius = point.distance(center);
    if (!sphere_identity(source, center, radius))
        return false;
    const double sign = sphere_orientation(source, center);
    const double factor = 1.0 + sign * distance / radius;
    if (std::abs(factor) <= 128.0 * Epsilon)
        throw std::runtime_error("Surface offset collapses to a point");
    result = source;
    result.m_mesh.clear();
    for (int i = 0; i < source.cv_count(0); ++i)
        for (int j = 0; j < source.cv_count(1); ++j)
            surface_control(
                result,
                i,
                j,
                center + (source.get_cv(i, j) - center) * factor
            );
    return true;
}

struct SurfaceField {
    mutable nurbs_detail::SurfaceEvaluator source;
    double distance;

    SurfaceField(const NurbsSurface& surface, double offset) : source(surface), distance(offset) {
    }

    Point at(double u, double v) const {
        const std::vector<Vector> derivatives = source.evaluate(u, v, 2);
        const Vector& su = derivatives[3];
        const Vector& sv = derivatives[1];
        const Vector area = su.cross(sv);
        const double area_length = vector_length(area);
        if (!std::isfinite(area_length) || area_length <= 128.0 * Epsilon * vector_length(su) * vector_length(sv))
            throw std::runtime_error("Surface offset has an undefined normal; a unique pole limit is required");
        const Vector normal = area / area_length;
        const Vector au = derivatives[5].cross(sv) + su.cross(derivatives[4]);
        const Vector av = derivatives[4].cross(sv) + su.cross(derivatives[2]);
        const Vector ou = su + (au - normal * normal.dot(au)) * (distance / area_length);
        const Vector ov = sv + (av - normal * normal.dot(av)) * (distance / area_length);
        const double offset_area = vector_length(ou.cross(ov));
        if (!std::isfinite(offset_area) || offset_area <= 128.0 * Epsilon * area_length)
            throw std::runtime_error("Surface offset has a focal singularity or collapse");
        const Vector position = derivatives[0] + normal * distance;
        for (int d = 0; d < 3; ++d)
            if (!std::isfinite(position[d]))
                throw std::runtime_error("Surface offset evaluation overflowed");
        return {position[0], position[1], position[2]};
    }
};

std::vector<double> initial_parameters(const std::vector<double>& spans) {
    std::vector<double> parameters;
    for (size_t i = 0; i + 1 < spans.size(); ++i)
        for (int j = 0; j < 4; ++j)
            parameters.push_back(spans[i] + (spans[i + 1] - spans[i]) * j / 4.0);
    parameters.push_back(spans.back());
    return parameters;
}

NurbsSurface interpolate_surface(
    const SurfaceField& field,
    const std::vector<double>& u,
    const std::vector<double>& v
) {
    std::vector<NurbsCurve> rows;
    for (double a : u) {
        std::vector<Point> points;
        for (double b : v)
            points.push_back(field.at(a, b));
        rows.push_back(NurbsCurve::create_interpolated(points, v));
        if (!rows.back().is_valid())
            throw std::runtime_error("Surface offset interpolation is ill-conditioned");
    }
    std::vector<NurbsCurve> columns;
    for (int j = 0; j < rows[0].cv_count(); ++j) {
        std::vector<Point> points;
        for (const NurbsCurve& row : rows)
            points.push_back(row.get_cv(j));
        columns.push_back(NurbsCurve::create_interpolated(points, u));
        if (!columns.back().is_valid())
            throw std::runtime_error("Surface offset interpolation is ill-conditioned");
    }
    NurbsSurface result(
        3,
        false,
        4,
        4,
        columns[0].cv_count(),
        rows[0].cv_count()
    );
    result.m_nurbsknot[0] = columns[0].m_nurbsknot;
    result.m_nurbsknot[1] = rows[0].m_nurbsknot;
    for (int i = 0; i < result.cv_count(0); ++i)
        for (int j = 0; j < result.cv_count(1); ++j)
            result.set_cv(i, j, columns[j].get_cv(i));
    return result;
}

std::vector<double> refine_parameters(const std::vector<double>& parameters, const std::vector<bool>& split) {
    std::vector<double> result;
    for (size_t i = 0; i + 1 < parameters.size(); ++i) {
        result.push_back(parameters[i]);
        if (split[i]) {
            const double middle = (parameters[i] + parameters[i + 1]) * 0.5;
            if (middle <= parameters[i] || middle >= parameters[i + 1])
                throw std::runtime_error("Surface offset exhausted parameter resolution");
            result.push_back(middle);
        }
    }
    result.push_back(parameters.back());
    return result;
}

bool refine_surface(
    const SurfaceField& field,
    const NurbsSurface& result,
    double tolerance,
    std::vector<double>& u,
    std::vector<double>& v
) {
    nurbs_detail::SurfaceEvaluator evaluation(result);
    std::vector<bool> split_u(u.size() - 1, false);
    std::vector<bool> split_v(v.size() - 1, false);
    bool passed = true;
    for (size_t i = 0; i + 1 < u.size(); ++i)
        for (size_t j = 0; j + 1 < v.size(); ++j)
            for (int a = 1; a < 4; ++a)
                for (int b = 1; b < 4; ++b) {
                    const double x = u[i] + (u[i + 1] - u[i]) * a / 4.0;
                    const double y = v[j] + (v[j + 1] - v[j]) * b / 4.0;
                    if (field.at(x, y).distance(evaluation.point_at(x, y)) <= tolerance * 0.25)
                        continue;
                    split_u[i] = true;
                    split_v[j] = true;
                    passed = false;
                }
    if (!passed) {
        u = refine_parameters(u, split_u);
        v = refine_parameters(v, split_v);
    }
    return passed;
}

}

NurbsCurve curve(
    const NurbsCurve& source,
    double distance,
    const Vector& normal,
    double tolerance
) {
    validate_options(distance, tolerance);
    NurbsCurve basis = source;
    validate_curve(basis, tolerance);
    if (distance == 0.0)
        return source;
    const double length = normal.magnitude();
    if (!std::isfinite(length) || length <= 0.0)
        throw std::invalid_argument("Curve offset requires a finite nonzero normal");
    const CurveField field{basis, distance, normal / length};
    NurbsCurve result;
    if (exact_curve(basis, distance, field.normal, result))
        return result;
    const std::vector<double> parameters = initial_parameters(basis.get_span_vector());
    std::vector<Point> controls;
    std::vector<double> knots;
    for (size_t i = 0; i + 1 < parameters.size(); ++i)
        append_curve_piece(field, {parameters[i], parameters[i + 1], 0}, tolerance, controls, knots);
    result = NurbsCurve(3, false, 4, static_cast<int>(controls.size()));
    result.m_nurbsknot = knots;
    for (int i = 0; i < result.cv_count(); ++i)
        result.set_cv(i, controls[i]);
    if (!result.is_valid())
        throw std::runtime_error("Curve offset reconstruction failed");
    return result;
}

NurbsSurface surface(const NurbsSurface& source, double distance, double tolerance) {
    validate_options(distance, tolerance);
    NurbsSurface basis = source;
    validate_surface(basis, tolerance);
    if (distance == 0.0)
        return source;
    check_surface_continuity(basis);
    NurbsSurface result;
    if (exact_surface(basis, distance, tolerance, result))
        return result;
    if (exact_cylinder(basis, distance, result))
        return result;
    const SurfaceField field{basis, distance};
    std::vector<double> u = initial_parameters(basis.get_span_vector(0));
    std::vector<double> v = initial_parameters(basis.get_span_vector(1));
    for (int iteration = 0; iteration < 12; ++iteration) {
        if (u.size() > 257 || v.size() > 257)
            break;
        result = interpolate_surface(field, u, v);
        if (refine_surface(field, result, tolerance, u, v))
            return result;
    }
    throw std::runtime_error("Surface offset refinement cannot meet sampled tolerance");
}

}
