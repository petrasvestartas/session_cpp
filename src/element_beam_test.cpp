#include "mini_test.h"
#include "element.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// ElementBeam
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("ElementBeam", "Constructor") {
    // uncomment #include "element_beam.h"
    // uncomment #include "mesh.h"
    ElementBeam b(0.1, 0.2, 3.0, "beam1");

    auto& geo = b.geometry();
    std::string name = b.name;
    const std::string& guid = b.guid();
    std::string bstr = b.str();
    std::string brepr = b.repr();

    ElementBeam bcopy = b.duplicate();
    ElementBeam b2(0.1, 0.2, 3.0, "beam1");
    ElementBeam b3(0.1, 0.2, 5.0, "beam1");

    MINI_CHECK(name == "beam1");
    MINI_CHECK(!guid.empty());
    MINI_CHECK(std::holds_alternative<Mesh>(geo));
    MINI_CHECK(b.width() == 0.1);
    MINI_CHECK(b.depth() == 0.2);
    MINI_CHECK(b.length() == 3.0);
    MINI_CHECK(bstr == "ElementBeam(beam1, 0.1, 0.2, 3)");
    MINI_CHECK(brepr == "ElementBeam(" + guid + ", beam1, 0.1, 0.2, 3)");
    MINI_CHECK(bcopy == b && bcopy.guid() != b.guid());
    MINI_CHECK(b == b2);
    MINI_CHECK(b != b3);
}

MINI_TEST("ElementBeam", "Setters") {
    // uncomment #include "element_beam.h"
    ElementBeam b;
    b.set_width(0.15);
    b.set_depth(0.3);
    b.set_length(5.0);

    MINI_CHECK(b.width() == 0.15);
    MINI_CHECK(b.depth() == 0.3);
    MINI_CHECK(b.length() == 5.0);
    MINI_CHECK(b.has_geometry());
}

MINI_TEST("ElementBeam", "Center Line") {
    // uncomment #include "element_beam.h"
    // uncomment #include "line.h"
    ElementBeam b(0.1, 0.2, 5.0);
    Line cl = b.center_line();

    MINI_CHECK(TOLERANCE.is_close(cl.start()[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(cl.end()[2], 5.0));
}

MINI_TEST("ElementBeam", "Extend") {
    // uncomment #include "element_beam.h"
    ElementBeam b(0.1, 0.2, 3.0);
    b.extend(0.5);

    MINI_CHECK(TOLERANCE.is_close(b.length(), 4.0));
}

MINI_TEST("ElementBeam", "AABB") {
    // uncomment #include "element_beam.h"
    // uncomment #include "obb.h"
    ElementBeam b(0.1, 0.2, 3.0);
    OBB aabb = b.aabb();

    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[0], 0.05));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[1], 0.1));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[2], 1.5));
}

MINI_TEST("ElementBeam", "Compute Point") {
    // uncomment #include "element_beam.h"
    // uncomment #include "point.h"
    ElementBeam b(0.1, 0.2, 3.0);
    Point pt = b.point();

    MINI_CHECK(TOLERANCE.is_close(pt[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pt[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pt[2], 1.5));
}

MINI_TEST("ElementBeam", "Session Geometry") {
    // uncomment #include "element_beam.h"
    // uncomment #include "mesh.h"
    // uncomment #include "xform.h"
    ElementBeam b(0.1, 0.2, 3.0);
    b.session_transformation = Xform::translation(10.0, 0.0, 0.0);
    auto sg = b.session_geometry();

    MINI_CHECK(std::holds_alternative<Mesh>(sg));
    auto& mesh = std::get<Mesh>(sg);
    double min_x = 1e9;
    for (const auto& [k, v] : mesh.vertex) if (v.x < min_x) min_x = v.x;
    MINI_CHECK(min_x > 9.0);
}

MINI_TEST("ElementBeam", "Json Roundtrip") {
    // uncomment #include "element_beam.h"
    // uncomment #include "xform.h"
    ElementBeam b(0.15, 0.3, 5.0, "json_beam");
    b.session_transformation = Xform::translation(1.0, 2.0, 3.0);

    std::string fname = "serialization/test_beam_element.json";
    b.file_json_dump(fname);
    ElementBeam loaded = ElementBeam::file_json_load(fname);

    MINI_CHECK(loaded.name == "json_beam");
    MINI_CHECK(TOLERANCE.is_close(loaded.width(), 0.15));
    MINI_CHECK(TOLERANCE.is_close(loaded.depth(), 0.3));
    MINI_CHECK(TOLERANCE.is_close(loaded.length(), 5.0));
}

MINI_TEST("ElementBeam", "Protobuf Roundtrip") {
    // uncomment #include "element_beam.h"
    // uncomment #include "xform.h"
    ElementBeam b(0.15, 0.3, 5.0, "proto_beam");
    b.session_transformation = Xform::translation(1.0, 2.0, 3.0);

    std::string path = "serialization/test_beam_element.bin";
    b.pb_dump(path);
    ElementBeam loaded = ElementBeam::pb_load(path);

    MINI_CHECK(loaded.name == "proto_beam");
    MINI_CHECK(TOLERANCE.is_close(loaded.width(), 0.15));
    MINI_CHECK(TOLERANCE.is_close(loaded.depth(), 0.3));
    MINI_CHECK(TOLERANCE.is_close(loaded.length(), 5.0));
}

///////////////////////////////////////////////////////////////////////////////////////////
// ElementBeam - Polylines/Planes/Edge Vectors/Axis
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("ElementBeam", "Polylines") {
    ElementBeam b(0.1, 0.2, 3.0);
    auto pls = b.polylines();
    MINI_CHECK(pls.size() == 6);
    for (const auto& pl : pls) MINI_CHECK(pl.point_count() == 5);
}

MINI_TEST("ElementBeam", "Planes") {
    ElementBeam b(0.1, 0.2, 3.0);
    auto pls = b.planes();
    MINI_CHECK(pls.size() == 6);
    MINI_CHECK(TOLERANCE.is_close(pls[0].z_axis()[2], -1.0));
    MINI_CHECK(TOLERANCE.is_close(pls[1].z_axis()[2], 1.0));
}

MINI_TEST("ElementBeam", "Edge Vectors") {
    ElementBeam b(0.1, 0.2, 3.0);
    auto evs = b.edge_vectors();
    MINI_CHECK(evs.size() == 12);
}

MINI_TEST("ElementBeam", "Axis") {
    ElementBeam b(0.1, 0.2, 5.0);
    auto ax = b.axis();
    MINI_CHECK(ax.has_value());
    MINI_CHECK(TOLERANCE.is_close(ax->start()[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(ax->end()[2], 5.0));
}

} // namespace session_cpp
