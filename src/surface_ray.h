#pragma once

#include "nurbs_implicit.h"
#include <array>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace session_cpp::surface_ray {

using Rectangle = std::array<double, 4>; // U minimum/maximum, V minimum/maximum.

struct Root {
    Point point;
    double u;
    double v;
    double distance; // Signed world distance along the input direction.
    Rectangle enclosure; // Contains the exact root, within the unique chart.
    Rectangle chart; // Original-domain rectangle containing exactly one root.
    std::array<double, 2> distance_enclosure;
    int crossing; // Certified sign of (Su cross Sv) dot direction, before face orientation.
};

/// A surface on a recognised plane, sphere, cylinder, cone or torus, laid out once for many rays: its Bezier patches, the box of each about the patches' origin, and a bound on the implicit form over the surface.
struct Form {
    nurbs_implicit::Implicit implicit;
    nurbs_implicit::Patches patches;
    std::vector<std::array<double, 6>> boxes;
    double band = std::numeric_limits<double>::infinity(); // The implicit form is at most this in size on the surface.
    double gauge = 0.0; // The least slope of the implicit form across its zero set.
    Form(const NurbsSurface& surface, const nurbs_implicit::Implicit& recognised);
};

struct Options {
    std::size_t max_cells = 200000;
    int max_depth = 96;
    bool include_origin = false; // Keep certified roots whose signed distance enclosure contains zero.
    std::optional<Form>* form = nullptr; // The surface's form, recognised on the first ray that needs it and kept for the next: roots are then found on the polynomial along the ray first.
    const NurbsSurface* layout = nullptr; // An unchanged copy of the surface that outlives the form, for the form to be laid out on.
};

struct Result {
    std::vector<Root> roots;
    bool complete = true;
    std::string failure;
    std::size_t cells = 0;
};

/// Certify regular isolated forward-ray roots on a continuous positive-rational support.
/// Every completed parameter cell is excluded or contained in a proven unique chart.
/// Natural-boundary, tangent, coincident and unresolved roots return complete=false.
/// Origin-straddling roots require include_origin for subsequent trim classification.
Result intersect(const NurbsSurface& surface, const Point& origin, const Vector& direction,
    double tolerance = 1e-7, const Options& options = {});

}
