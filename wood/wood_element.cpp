#include "wood_element.h"

#include "../src/line.h"
#include "../src/point.h"

namespace wood_session {

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

} // namespace wood_session
