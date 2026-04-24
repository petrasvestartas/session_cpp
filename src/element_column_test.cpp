#include "mini_test.h"
#include "element.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// ElementColumn
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("ElementColumn", "Constructor") {
    // uncomment #include "element_column.h"
    // uncomment #include "mesh.h"
    ElementColumn c(0.4, 0.4, 3.0, "col1");

    auto& geo = c.geometry();
    std::string name = c.name;
    const std::string& guid = c.guid();
    std::string cstr = c.str();
    std::string crepr = c.repr();

    ElementColumn ccopy = c.duplicate();
    ElementColumn c2(0.4, 0.4, 3.0, "col1");
    ElementColumn c3(0.5, 0.4, 3.0, "col1");

    MINI_CHECK(name == "col1");
    MINI_CHECK(!guid.empty());
    MINI_CHECK(std::holds_alternative<Mesh>(geo));
    MINI_CHECK(c.width() == 0.4);
    MINI_CHECK(c.depth() == 0.4);
    MINI_CHECK(c.height() == 3.0);
    MINI_CHECK(cstr == "ElementColumn(col1, 0.4, 0.4, 3)");
    MINI_CHECK(crepr == "ElementColumn(" + guid + ", col1, 0.4, 0.4, 3)");
    MINI_CHECK(ccopy == c && ccopy.guid() != c.guid());
    MINI_CHECK(c == c2);
    MINI_CHECK(c != c3);
}

MINI_TEST("ElementColumn", "Setters") {
    // uncomment #include "element_column.h"
    ElementColumn c;
    c.set_width(0.5);
    c.set_depth(0.6);
    c.set_height(4.0);

    MINI_CHECK(c.width() == 0.5);
    MINI_CHECK(c.depth() == 0.6);
    MINI_CHECK(c.height() == 4.0);
    MINI_CHECK(c.has_geometry());
}

MINI_TEST("ElementColumn", "Center Line") {
    // uncomment #include "element_column.h"
    // uncomment #include "line.h"
    ElementColumn c(0.4, 0.4, 5.0);
    Line cl = c.center_line();

    MINI_CHECK(TOLERANCE.is_close(cl.start()[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(cl.end()[2], 5.0));
}

MINI_TEST("ElementColumn", "Extend") {
    // uncomment #include "element_column.h"
    ElementColumn c(0.4, 0.4, 3.0);
    c.extend(0.5);

    MINI_CHECK(TOLERANCE.is_close(c.height(), 4.0));
}

MINI_TEST("ElementColumn", "AABB") {
    // uncomment #include "element_column.h"
    // uncomment #include "obb.h"
    ElementColumn c(0.4, 0.4, 3.0);
    OBB aabb = c.aabb();

    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[0], 0.2));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[1], 0.2));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[2], 1.5));
}

MINI_TEST("ElementColumn", "Compute Point") {
    // uncomment #include "element_column.h"
    // uncomment #include "point.h"
    ElementColumn c(0.4, 0.4, 3.0);
    Point pt = c.point();

    MINI_CHECK(TOLERANCE.is_close(pt[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pt[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pt[2], 1.5));
}

MINI_TEST("ElementColumn", "Session Geometry") {
    // uncomment #include "element_column.h"
    // uncomment #include "mesh.h"
    // uncomment #include "xform.h"
    ElementColumn c(0.4, 0.4, 3.0);
    c.session_transformation = Xform::translation(10.0, 0.0, 0.0);
    auto sg = c.session_geometry();

    MINI_CHECK(std::holds_alternative<Mesh>(sg));
    auto& mesh = std::get<Mesh>(sg);
    double min_x = 1e9;
    for (const auto& [k, v] : mesh.vertex) if (v.x < min_x) min_x = v.x;
    MINI_CHECK(min_x > 9.0);
}

MINI_TEST("ElementColumn", "Json Roundtrip") {
    // uncomment #include "element_column.h"
    // uncomment #include "xform.h"
    ElementColumn c(0.5, 0.6, 4.0, "json_col");
    c.session_transformation = Xform::translation(1.0, 2.0, 3.0);

    std::string fname = "serialization/test_column_element.json";
    c.file_json_dump(fname);
    ElementColumn loaded = ElementColumn::file_json_load(fname);

    MINI_CHECK(loaded.name == "json_col");
    MINI_CHECK(TOLERANCE.is_close(loaded.width(), 0.5));
    MINI_CHECK(TOLERANCE.is_close(loaded.depth(), 0.6));
    MINI_CHECK(TOLERANCE.is_close(loaded.height(), 4.0));
}

MINI_TEST("ElementColumn", "Protobuf Roundtrip") {
    // uncomment #include "element_column.h"
    // uncomment #include "xform.h"
    ElementColumn c(0.5, 0.6, 4.0, "proto_col");
    c.session_transformation = Xform::translation(1.0, 2.0, 3.0);

    std::string path = "serialization/test_column_element.bin";
    c.pb_dump(path);
    ElementColumn loaded = ElementColumn::pb_load(path);

    MINI_CHECK(loaded.name == "proto_col");
    MINI_CHECK(TOLERANCE.is_close(loaded.width(), 0.5));
    MINI_CHECK(TOLERANCE.is_close(loaded.depth(), 0.6));
    MINI_CHECK(TOLERANCE.is_close(loaded.height(), 4.0));
}

///////////////////////////////////////////////////////////////////////////////////////////
// ElementColumn - Polylines/Planes/Edge Vectors/Axis
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("ElementColumn", "Polylines") {
    ElementColumn c(0.4, 0.4, 3.0);
    auto pls = c.polylines();
    MINI_CHECK(pls.size() == 6);
    for (const auto& pl : pls) MINI_CHECK(pl.point_count() == 5);
}

MINI_TEST("ElementColumn", "Planes") {
    ElementColumn c(0.4, 0.4, 3.0);
    auto pls = c.planes();
    MINI_CHECK(pls.size() == 6);
    MINI_CHECK(TOLERANCE.is_close(pls[0].z_axis()[2], -1.0));
    MINI_CHECK(TOLERANCE.is_close(pls[1].z_axis()[2], 1.0));
}

MINI_TEST("ElementColumn", "Edge Vectors") {
    ElementColumn c(0.4, 0.4, 3.0);
    auto evs = c.edge_vectors();
    MINI_CHECK(evs.size() == 12);
}

MINI_TEST("ElementColumn", "Axis") {
    ElementColumn c(0.4, 0.4, 5.0);
    auto ax = c.axis();
    MINI_CHECK(ax.has_value());
    MINI_CHECK(TOLERANCE.is_close(ax->start()[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(ax->end()[2], 5.0));
}

} // namespace session_cpp
