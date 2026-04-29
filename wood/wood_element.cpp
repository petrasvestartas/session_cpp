#include "wood_element.h"

#include "../src/line.h"
#include "../src/plane.h"
#include "../src/point.h"
#include "../src/polyline.h"
#include "../src/vector.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace wood_session {

using session_cpp::Plane;
using session_cpp::Point;
using session_cpp::Polyline;
using session_cpp::Vector;

WoodJoint::WoodJoint()
    : el_ids{0, 0}
    , face_ids{ {{0,0}}, {{0,0}} }
    , joint_type{0}
    , joint_area{session_cpp::Polyline(std::vector<session_cpp::Point>{})}
    , joint_lines{
        session_cpp::Line::from_points(session_cpp::Point(0,0,0), session_cpp::Point(0,0,0)),
        session_cpp::Line::from_points(session_cpp::Point(0,0,0), session_cpp::Point(0,0,0)),
      }
    , divisions{1}
    , shift{0.5}
    , length{0}
    , division_length{0.0}
    , scale{1.0, 1.0, 1.0}
    , unit_scale{false}
    , unit_scale_distance{0.0}
    , link{false}
    , no_orient{false}
    , dbg_coplanar{0}
    , dbg_boolean{0}
{}

WoodElement::WoodElement()
    : reversed{false}
    , thickness{0.0}
{}

WoodElement::WoodElement(const Polyline& bot, const Polyline& top)
    : reversed{false}
    , thickness{0.0}
{
    std::vector<Point> pp0 = bot.get_points();
    std::vector<Point> pp1 = top.get_points();

    // Strip closing point if present
    auto strip = [](std::vector<Point>& v) {
        if (v.size() > 3) {
            auto& f = v.front(); auto& l = v.back();
            if (std::abs(f[0]-l[0])<1e-6 && std::abs(f[1]-l[1])<1e-6 && std::abs(f[2]-l[2])<1e-6)
                v.pop_back();
        }
    };

    // Wood orientation check (`wood_main.cpp:73-110`): transform the
    // concatenated `twoPolylines = pp[i] + pp[i+1]` through
    // `plane_to_xy(average_plane(pp[i]))` and reverse both polylines if
    // `twoPolylines.back().z() > 0`. The transformed z of a point p equals
    // `dot(p - origin, z_axis)` when the frame is orthonormal, so we
    // compute that directly instead of materialising a 4×4 transform.
    //
    // Frame: origin = centroid(pp0), z_axis = average_normal(pp0). We need
    // only the z-component; the x/y components are unused here.
    Vector normal = Vector::average_normal(pp0);
    auto pp0_open = pp0;
    strip(pp0_open);
    Point c0 = Point::centroid(pp0_open);
    Point last_p1 = pp1.back();
    double last_z = (last_p1[0]-c0[0])*normal[0]
                  + (last_p1[1]-c0[1])*normal[1]
                  + (last_p1[2]-c0[2])*normal[2];
    if (last_z > 0) {
        std::reverse(pp0.begin(), pp0.end());
        std::reverse(pp1.begin(), pp1.end());
        normal = Vector::average_normal(pp0);
        reversed = true;
    }

    // Wood uses the CLOSED polyline for side iteration: j = 0..size()-2.
    // For a closed quad (5 points), that gives 4 side faces.
    // Compute normal/centroid from open points, but iterate closed points for sides.
    size_t n_sides = pp0.size() > 1 ? pp0.size() - 1 : 0;

    polylines.resize(2 + n_sides, Polyline(std::vector<Point>{}));
    polylines[0] = Polyline(pp0);
    polylines[1] = Polyline(pp1);

    // Strip for centroid computation only
    auto pp0_stripped = pp0;
    auto pp1_stripped = pp1;
    strip(pp0_stripped);
    strip(pp1_stripped);
    Point cen0 = Point::centroid(pp0_stripped);
    Point cen1 = Point::centroid(pp1_stripped);
    planes.resize(2 + n_sides);
    Vector neg_normal(-normal[0],-normal[1],-normal[2]);
    planes[0] = Plane::from_point_normal(cen0, normal);
    planes[1] = Plane::from_point_normal(cen1, neg_normal);
    thickness = Point::distance(cen0, planes[1].project(cen0));

    // Side planes from 3 points on the CLOSED polyline: (pp0[j+1], pp0[j], pp1[j+1])
    for (size_t j = 0; j < n_sides; j++) {
        // 3-point plane: normal = (pp0[j]-pp0[j+1]) × (pp1[j+1]-pp0[j+1])
        double ax = pp0[j][0]-pp0[j+1][0];
        double ay = pp0[j][1]-pp0[j+1][1];
        double az = pp0[j][2]-pp0[j+1][2];
        double bx = pp1[j+1][0]-pp0[j+1][0];
        double by = pp1[j+1][1]-pp0[j+1][1];
        double bz = pp1[j+1][2]-pp0[j+1][2];
        double nx = ay*bz-az*by;
        double ny = az*bx-ax*bz;
        double nz = ax*by-ay*bx;
        // Build plane with CGAL-compatible base1/base2 axes.
        Point side_origin = pp0[j+1];
        double anx = std::abs(nx), any = std::abs(ny), anz = std::abs(nz);
        Vector sb1;
        if (anx < 1e-12)      sb1 = Vector(1,0,0);
        else if (any < 1e-12) sb1 = Vector(0,1,0);
        else if (anz < 1e-12) sb1 = Vector(0,0,1);
        else if (anx<=any && anx<=anz) sb1 = Vector(0,-nz,ny);
        else if (any<=anx && any<=anz) sb1 = Vector(-nz,0,nx);
        else                           sb1 = Vector(-ny,nx,0);
        Vector snv(nx,ny,nz);
        Vector sb2 = snv.cross(sb1);
        sb1.normalize_self();
        sb2.normalize_self();
        snv.normalize_self();
        planes[2+j] = Plane(side_origin, sb1, sb2, snv);
        polylines[2+j] = Polyline(std::vector<Point>{
            pp0[j], pp0[j+1], pp1[j+1], pp1[j], pp0[j]});
    }
}

} // namespace wood_session
