#pragma once

#include "mesh.h"
#include "obb.h"
#include "point.h"
#include <functional>

namespace session_cpp {

enum class TpmsType : int {
    GYROID = 0, SCHWARZ_P = 1, DIAMOND = 2, NEOVIUS = 3, IWP = 4,
    LIDINOID = 5, FISCHER_KOCH_S = 6, FRD = 7, PMY = 8
};

enum class TpmsMode : int {
    SOLID = 0, SHEET = 1, SHELL = 2
};

class MeshIso {
public:
    static double eval(TpmsType type, double x, double y, double z, double period = 1.0);

    static Mesh from_tpms(TpmsType type, const OBB& box,
                           int nx, int ny, int nz,
                           double isovalue = 0.0, double period = 1.0,
                           TpmsMode mode = TpmsMode::SOLID, double thickness = 0.2);

    static Mesh from_function(std::function<double(double,double,double)> fn,
                               const OBB& box, int nx, int ny, int nz,
                               double isovalue = 0.0);

    static double sdf_sphere(double cx, double cy, double cz, double r,
                              double x, double y, double z);

    static double sdf_box(double cx, double cy, double cz,
                           double hx, double hy, double hz,
                           double x, double y, double z);

    static double sdf_capsule(const Point& p0, const Point& p1, double r,
                               double x, double y, double z);

    static double sdf_torus(double cx, double cy, double cz,
                             double major_r, double minor_r,
                             double x, double y, double z);

    static double sdf_plane(double nx, double ny, double nz, double d,
                             double x, double y, double z);

    static double smooth_union(double a, double b, double k = 8.0);
    static double smooth_subtract(double a, double b, double k = 8.0);
    static double smooth_intersect(double a, double b, double k = 8.0);
};

} // namespace session_cpp
