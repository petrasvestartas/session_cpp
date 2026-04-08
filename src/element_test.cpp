#include "mini_test.h"
#include "element.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// Element
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("Element", "Constructor") {
    // uncomment #include "element.h"
    // uncomment #include "brep.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m, "test_element");

    auto& geo = e.geometry();
    std::string name = e.name;
    const std::string& guid = e.guid();
    bool dirty = e.is_dirty();

    std::string estr = e.str();
    std::string erepr = e.repr();

    Element ecopy = e.duplicate();

    Element e2(Mesh(), "test_element");
    Element e3(BRep(), "other");

    MINI_CHECK(name == "test_element");
    MINI_CHECK(!guid.empty());
    MINI_CHECK(dirty);
    MINI_CHECK(std::holds_alternative<Mesh>(geo));
    MINI_CHECK(estr == "Element(test_element, Mesh)");
    MINI_CHECK(erepr == "Element(" + guid + ", test_element, Mesh)");
    MINI_CHECK(ecopy == e && ecopy.guid() != e.guid());
    MINI_CHECK(e == e2);
    MINI_CHECK(e != e3);
}

MINI_TEST("Element", "Session Transformation") {
    // uncomment #include "element.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    // uncomment #include "xform.h"
    Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    Xform xf = Xform::translation(10.0, 20.0, 30.0);
    e.session_transformation = xf;

    MINI_CHECK(e.is_dirty());
    MINI_CHECK(e.session_transformation == xf);
}

MINI_TEST("Element", "Add Feature") {
    // uncomment #include "element.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);

    auto my_feature = [](Mesh geo) -> Mesh { return geo; };
    e.add_feature(my_feature);

    MINI_CHECK(e.is_dirty());
    MINI_CHECK(e.features_count() == 1);
}

MINI_TEST("Element", "Aabb") {
    // uncomment #include "element.h"
    // uncomment #include "mesh.h"
    // uncomment #include "obb.h"
    // uncomment #include "point.h"
    Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    OBB aabb = e.aabb();

    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[0], 0.5));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[1], 0.5));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[2], 0.0));
}

MINI_TEST("Element", "OBB") {
    // uncomment #include "element.h"
    // uncomment #include "mesh.h"
    // uncomment #include "obb.h"
    // uncomment #include "point.h"
    Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    OBB obb = e.obb();

    MINI_CHECK(TOLERANCE.is_close(obb.half_size[0], 0.5));
    MINI_CHECK(TOLERANCE.is_close(obb.half_size[1], 0.5));
}

MINI_TEST("Element", "Session Geometry") {
    // uncomment #include "element.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    // uncomment #include "xform.h"
    Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    e.session_transformation = Xform::translation(10.0, 0.0, 0.0);
    auto sg = e.session_geometry();

    MINI_CHECK(std::holds_alternative<Mesh>(sg));
    auto& mesh = std::get<Mesh>(sg);
    auto verts_it = mesh.vertex.begin();
    MINI_CHECK(TOLERANCE.is_close(verts_it->second.x, 10.0));
    ++verts_it;
    MINI_CHECK(TOLERANCE.is_close(verts_it->second.x, 11.0));
    MINI_CHECK(&std::get<Mesh>(e.geometry()) != &mesh);
}

MINI_TEST("Element", "Reset") {
    // uncomment #include "element.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(2, 0, 0),
            Point(2, 2, 0),
            Point(0, 2, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    auto _ = e.aabb();
    auto _2 = e.point();
    e.reset();

    MINI_CHECK(e.is_dirty());
    MINI_CHECK(!e.cached_aabb().has_value());
    MINI_CHECK(!e.cached_obb().has_value());
    MINI_CHECK(!e.cached_collision_mesh().has_value());
    MINI_CHECK(!e.cached_point().has_value());
}

MINI_TEST("Element", "Compute Point") {
    // uncomment #include "element.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(2, 0, 0),
            Point(2, 2, 0),
            Point(0, 2, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    Point pt = e.point();

    MINI_CHECK(TOLERANCE.is_close(pt[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(pt[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(pt[2], 0.0));
}

MINI_TEST("Element", "Brep Aabb") {
    // uncomment #include "element.h"
    // uncomment #include "brep.h"
    // uncomment #include "obb.h"
    // uncomment #include "point.h"
    BRep b = BRep::create_box(2.0, 3.0, 4.0);
    Element e(b, "brep_element");
    OBB aabb = e.aabb();
    Point pt = e.point();

    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[1], 1.5));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[2], 2.0));
    MINI_CHECK(TOLERANCE.is_close(pt[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pt[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pt[2], 0.0));
}

MINI_TEST("Element", "Json Roundtrip") {
    // uncomment #include "element.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    // uncomment #include "xform.h"
    Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m, "json_test");
    e.session_transformation = Xform::translation(1.0, 2.0, 3.0);

    std::string fname = "serialization/test_element.json";
    e.json_dump(fname);
    Element loaded = Element::json_load(fname);

    MINI_CHECK(loaded.name == "json_test");
    MINI_CHECK(std::holds_alternative<Mesh>(loaded.geometry()));
    MINI_CHECK(std::get<Mesh>(loaded.geometry()).vertex.size() == 4);
}

MINI_TEST("Element", "Protobuf Roundtrip") {
    // uncomment #include "element.h"
    // uncomment #include "brep.h"
    // uncomment #include "xform.h"
    BRep b = BRep::create_box(2.0, 3.0, 4.0);
    Element e(b, "proto_test");
    e.session_transformation = Xform::translation(1.0, 2.0, 3.0);

    std::string path = "serialization/test_element.bin";
    e.pb_dump(path);
    Element loaded = Element::pb_load(path);

    MINI_CHECK(loaded.name == "proto_test");
    MINI_CHECK(std::holds_alternative<BRep>(loaded.geometry()));
    MINI_CHECK(std::get<BRep>(loaded.geometry()).face_count() == 6);
    MINI_CHECK(std::get<BRep>(loaded.geometry()).vertex_count() == 8);
}

///////////////////////////////////////////////////////////////////////////////////////////
// ColumnElement
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("ColumnElement", "Constructor") {
    // uncomment #include "element_column.h"
    // uncomment #include "mesh.h"
    ColumnElement c(0.4, 0.4, 3.0, "col1");

    auto& geo = c.geometry();
    std::string name = c.name;
    const std::string& guid = c.guid();
    std::string cstr = c.str();
    std::string crepr = c.repr();

    ColumnElement ccopy = c.duplicate();
    ColumnElement c2(0.4, 0.4, 3.0, "col1");
    ColumnElement c3(0.5, 0.4, 3.0, "col1");

    MINI_CHECK(name == "col1");
    MINI_CHECK(!guid.empty());
    MINI_CHECK(std::holds_alternative<Mesh>(geo));
    MINI_CHECK(c.width() == 0.4);
    MINI_CHECK(c.depth() == 0.4);
    MINI_CHECK(c.height() == 3.0);
    MINI_CHECK(cstr == "ColumnElement(col1, 0.4, 0.4, 3)");
    MINI_CHECK(crepr == "ColumnElement(" + guid + ", col1, 0.4, 0.4, 3)");
    MINI_CHECK(ccopy == c && ccopy.guid() != c.guid());
    MINI_CHECK(c == c2);
    MINI_CHECK(c != c3);
}

MINI_TEST("ColumnElement", "Setters") {
    // uncomment #include "element_column.h"
    ColumnElement c;
    c.set_width(0.5);
    c.set_depth(0.6);
    c.set_height(4.0);

    MINI_CHECK(c.width() == 0.5);
    MINI_CHECK(c.depth() == 0.6);
    MINI_CHECK(c.height() == 4.0);
    MINI_CHECK(c.has_geometry());
}

MINI_TEST("ColumnElement", "Center Line") {
    // uncomment #include "element_column.h"
    // uncomment #include "line.h"
    ColumnElement c(0.4, 0.4, 5.0);
    Line cl = c.center_line();

    MINI_CHECK(TOLERANCE.is_close(cl.start()[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(cl.end()[2], 5.0));
}

MINI_TEST("ColumnElement", "Extend") {
    // uncomment #include "element_column.h"
    ColumnElement c(0.4, 0.4, 3.0);
    c.extend(0.5);

    MINI_CHECK(TOLERANCE.is_close(c.height(), 4.0));
}

MINI_TEST("ColumnElement", "Aabb") {
    // uncomment #include "element_column.h"
    // uncomment #include "obb.h"
    ColumnElement c(0.4, 0.4, 3.0);
    OBB aabb = c.aabb();

    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[0], 0.2));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[1], 0.2));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[2], 1.5));
}

MINI_TEST("ColumnElement", "Compute Point") {
    // uncomment #include "element_column.h"
    // uncomment #include "point.h"
    ColumnElement c(0.4, 0.4, 3.0);
    Point pt = c.point();

    MINI_CHECK(TOLERANCE.is_close(pt[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pt[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pt[2], 1.5));
}

MINI_TEST("ColumnElement", "Session Geometry") {
    // uncomment #include "element_column.h"
    // uncomment #include "mesh.h"
    // uncomment #include "xform.h"
    ColumnElement c(0.4, 0.4, 3.0);
    c.session_transformation = Xform::translation(10.0, 0.0, 0.0);
    auto sg = c.session_geometry();

    MINI_CHECK(std::holds_alternative<Mesh>(sg));
    auto& mesh = std::get<Mesh>(sg);
    double min_x = 1e9;
    for (const auto& [k, v] : mesh.vertex) if (v.x < min_x) min_x = v.x;
    MINI_CHECK(min_x > 9.0);
}

MINI_TEST("ColumnElement", "Json Roundtrip") {
    // uncomment #include "element_column.h"
    // uncomment #include "xform.h"
    ColumnElement c(0.5, 0.6, 4.0, "json_col");
    c.session_transformation = Xform::translation(1.0, 2.0, 3.0);

    std::string fname = "serialization/test_column_element.json";
    c.json_dump(fname);
    ColumnElement loaded = ColumnElement::json_load(fname);

    MINI_CHECK(loaded.name == "json_col");
    MINI_CHECK(TOLERANCE.is_close(loaded.width(), 0.5));
    MINI_CHECK(TOLERANCE.is_close(loaded.depth(), 0.6));
    MINI_CHECK(TOLERANCE.is_close(loaded.height(), 4.0));
}

MINI_TEST("ColumnElement", "Protobuf Roundtrip") {
    // uncomment #include "element_column.h"
    // uncomment #include "xform.h"
    ColumnElement c(0.5, 0.6, 4.0, "proto_col");
    c.session_transformation = Xform::translation(1.0, 2.0, 3.0);

    std::string path = "serialization/test_column_element.bin";
    c.pb_dump(path);
    ColumnElement loaded = ColumnElement::pb_load(path);

    MINI_CHECK(loaded.name == "proto_col");
    MINI_CHECK(TOLERANCE.is_close(loaded.width(), 0.5));
    MINI_CHECK(TOLERANCE.is_close(loaded.depth(), 0.6));
    MINI_CHECK(TOLERANCE.is_close(loaded.height(), 4.0));
}

///////////////////////////////////////////////////////////////////////////////////////////
// BeamElement
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("BeamElement", "Constructor") {
    // uncomment #include "element_beam.h"
    // uncomment #include "mesh.h"
    BeamElement b(0.1, 0.2, 3.0, "beam1");

    auto& geo = b.geometry();
    std::string name = b.name;
    const std::string& guid = b.guid();
    std::string bstr = b.str();
    std::string brepr = b.repr();

    BeamElement bcopy = b.duplicate();
    BeamElement b2(0.1, 0.2, 3.0, "beam1");
    BeamElement b3(0.1, 0.2, 5.0, "beam1");

    MINI_CHECK(name == "beam1");
    MINI_CHECK(!guid.empty());
    MINI_CHECK(std::holds_alternative<Mesh>(geo));
    MINI_CHECK(b.width() == 0.1);
    MINI_CHECK(b.depth() == 0.2);
    MINI_CHECK(b.length() == 3.0);
    MINI_CHECK(bstr == "BeamElement(beam1, 0.1, 0.2, 3)");
    MINI_CHECK(brepr == "BeamElement(" + guid + ", beam1, 0.1, 0.2, 3)");
    MINI_CHECK(bcopy == b && bcopy.guid() != b.guid());
    MINI_CHECK(b == b2);
    MINI_CHECK(b != b3);
}

MINI_TEST("BeamElement", "Setters") {
    // uncomment #include "element_beam.h"
    BeamElement b;
    b.set_width(0.15);
    b.set_depth(0.3);
    b.set_length(5.0);

    MINI_CHECK(b.width() == 0.15);
    MINI_CHECK(b.depth() == 0.3);
    MINI_CHECK(b.length() == 5.0);
    MINI_CHECK(b.has_geometry());
}

MINI_TEST("BeamElement", "Center Line") {
    // uncomment #include "element_beam.h"
    // uncomment #include "line.h"
    BeamElement b(0.1, 0.2, 5.0);
    Line cl = b.center_line();

    MINI_CHECK(TOLERANCE.is_close(cl.start()[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(cl.end()[2], 5.0));
}

MINI_TEST("BeamElement", "Extend") {
    // uncomment #include "element_beam.h"
    BeamElement b(0.1, 0.2, 3.0);
    b.extend(0.5);

    MINI_CHECK(TOLERANCE.is_close(b.length(), 4.0));
}

MINI_TEST("BeamElement", "Aabb") {
    // uncomment #include "element_beam.h"
    // uncomment #include "obb.h"
    BeamElement b(0.1, 0.2, 3.0);
    OBB aabb = b.aabb();

    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[0], 0.05));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[1], 0.1));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[2], 1.5));
}

MINI_TEST("BeamElement", "Compute Point") {
    // uncomment #include "element_beam.h"
    // uncomment #include "point.h"
    BeamElement b(0.1, 0.2, 3.0);
    Point pt = b.point();

    MINI_CHECK(TOLERANCE.is_close(pt[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pt[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pt[2], 1.5));
}

MINI_TEST("BeamElement", "Session Geometry") {
    // uncomment #include "element_beam.h"
    // uncomment #include "mesh.h"
    // uncomment #include "xform.h"
    BeamElement b(0.1, 0.2, 3.0);
    b.session_transformation = Xform::translation(10.0, 0.0, 0.0);
    auto sg = b.session_geometry();

    MINI_CHECK(std::holds_alternative<Mesh>(sg));
    auto& mesh = std::get<Mesh>(sg);
    double min_x = 1e9;
    for (const auto& [k, v] : mesh.vertex) if (v.x < min_x) min_x = v.x;
    MINI_CHECK(min_x > 9.0);
}

MINI_TEST("BeamElement", "Json Roundtrip") {
    // uncomment #include "element_beam.h"
    // uncomment #include "xform.h"
    BeamElement b(0.15, 0.3, 5.0, "json_beam");
    b.session_transformation = Xform::translation(1.0, 2.0, 3.0);

    std::string fname = "serialization/test_beam_element.json";
    b.json_dump(fname);
    BeamElement loaded = BeamElement::json_load(fname);

    MINI_CHECK(loaded.name == "json_beam");
    MINI_CHECK(TOLERANCE.is_close(loaded.width(), 0.15));
    MINI_CHECK(TOLERANCE.is_close(loaded.depth(), 0.3));
    MINI_CHECK(TOLERANCE.is_close(loaded.length(), 5.0));
}

MINI_TEST("BeamElement", "Protobuf Roundtrip") {
    // uncomment #include "element_beam.h"
    // uncomment #include "xform.h"
    BeamElement b(0.15, 0.3, 5.0, "proto_beam");
    b.session_transformation = Xform::translation(1.0, 2.0, 3.0);

    std::string path = "serialization/test_beam_element.bin";
    b.pb_dump(path);
    BeamElement loaded = BeamElement::pb_load(path);

    MINI_CHECK(loaded.name == "proto_beam");
    MINI_CHECK(TOLERANCE.is_close(loaded.width(), 0.15));
    MINI_CHECK(TOLERANCE.is_close(loaded.depth(), 0.3));
    MINI_CHECK(TOLERANCE.is_close(loaded.length(), 5.0));
}

///////////////////////////////////////////////////////////////////////////////////////////
// PlateElement
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("PlateElement", "Constructor") {
    // uncomment #include "element_plate.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    std::vector<Point> polygon = {
        Point(0, 0, 0),
        Point(2, 0, 0),
        Point(2, 2, 0),
        Point(0, 2, 0),
    };
    PlateElement p(polygon, 0.2, "plate1");

    auto& geo = p.geometry();
    std::string name = p.name;
    const std::string& guid = p.guid();
    std::string pstr = p.str();
    std::string prepr = p.repr();

    PlateElement pcopy = p.duplicate();
    PlateElement p2(polygon, 0.2, "plate1");
    PlateElement p3(polygon, 0.5, "plate1");

    MINI_CHECK(name == "plate1");
    MINI_CHECK(!guid.empty());
    MINI_CHECK(std::holds_alternative<Mesh>(geo));
    MINI_CHECK(p.polygon().size() == 4);
    MINI_CHECK(p.thickness() == 0.2);
    MINI_CHECK(pstr == "PlateElement(plate1, 4 pts, 0.2)");
    MINI_CHECK(prepr == "PlateElement(" + guid + ", plate1, 4 pts, 0.2)");
    MINI_CHECK(pcopy == p && pcopy.guid() != p.guid());
    MINI_CHECK(p == p2);
    MINI_CHECK(p != p3);
}

MINI_TEST("PlateElement", "Default Polygon") {
    // uncomment #include "element_plate.h"
    // uncomment #include "mesh.h"
    PlateElement p;

    MINI_CHECK(std::holds_alternative<Mesh>(p.geometry()));
    MINI_CHECK(p.polygon().size() == 4);
    MINI_CHECK(p.thickness() == 0.1);
}

MINI_TEST("PlateElement", "Setters") {
    // uncomment #include "element_plate.h"
    // uncomment #include "point.h"
    PlateElement p;
    p.set_thickness(0.3);
    p.set_polygon({
        Point(0, 0, 0),
        Point(3, 0, 0),
        Point(3, 3, 0),
        Point(0, 3, 0),
    });

    MINI_CHECK(p.thickness() == 0.3);
    MINI_CHECK(p.polygon().size() == 4);
    MINI_CHECK(p.has_geometry());
}

MINI_TEST("PlateElement", "Mesh Topology") {
    // uncomment #include "element_plate.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    std::vector<Point> polygon = {
        Point(0, 0, 0),
        Point(1, 0, 0),
        Point(1, 1, 0),
        Point(0, 1, 0),
    };
    PlateElement p(polygon, 0.5);
    auto& geo = std::get<Mesh>(p.geometry());

    MINI_CHECK(geo.vertex.size() == 8);
    MINI_CHECK(geo.face.size() == 6);
}

MINI_TEST("PlateElement", "Aabb") {
    // uncomment #include "element_plate.h"
    // uncomment #include "obb.h"
    // uncomment #include "point.h"
    std::vector<Point> polygon = {
        Point(0, 0, 0),
        Point(2, 0, 0),
        Point(2, 2, 0),
        Point(0, 2, 0),
    };
    PlateElement p(polygon, 0.2);
    OBB aabb = p.aabb();

    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[2], 0.1));
}

MINI_TEST("PlateElement", "Compute Point") {
    // uncomment #include "element_plate.h"
    // uncomment #include "point.h"
    std::vector<Point> polygon = {
        Point(0, 0, 0),
        Point(2, 0, 0),
        Point(2, 2, 0),
        Point(0, 2, 0),
    };
    PlateElement p(polygon, 0.2);
    Point pt = p.point();

    MINI_CHECK(TOLERANCE.is_close(pt[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(pt[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(pt[2], -0.1));
}

MINI_TEST("PlateElement", "Triangle Polygon") {
    // uncomment #include "element_plate.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    std::vector<Point> polygon = {
        Point(0, 0, 0),
        Point(1, 0, 0),
        Point(0.5, 1, 0),
    };
    PlateElement p(polygon, 0.1);
    auto& geo = std::get<Mesh>(p.geometry());

    MINI_CHECK(geo.vertex.size() == 6);
    MINI_CHECK(geo.face.size() == 5);
}

MINI_TEST("PlateElement", "Json Roundtrip") {
    // uncomment #include "element_plate.h"
    // uncomment #include "point.h"
    // uncomment #include "xform.h"
    std::vector<Point> polygon = {
        Point(0, 0, 0),
        Point(2, 0, 0),
        Point(2, 2, 0),
        Point(0, 2, 0),
    };
    PlateElement p(polygon, 0.3, "json_plate");
    p.session_transformation = Xform::translation(1.0, 2.0, 3.0);

    std::string fname = "serialization/test_plate_element.json";
    p.json_dump(fname);
    PlateElement loaded = PlateElement::json_load(fname);

    MINI_CHECK(loaded.name == "json_plate");
    MINI_CHECK(TOLERANCE.is_close(loaded.thickness(), 0.3));
    MINI_CHECK(loaded.polygon().size() == 4);
    MINI_CHECK(TOLERANCE.is_close(loaded.polygon()[1][0], 2.0));
}

MINI_TEST("PlateElement", "Protobuf Roundtrip") {
    // uncomment #include "element_plate.h"
    // uncomment #include "point.h"
    // uncomment #include "xform.h"
    std::vector<Point> polygon = {
        Point(0, 0, 0),
        Point(2, 0, 0),
        Point(2, 2, 0),
        Point(0, 2, 0),
    };
    PlateElement p(polygon, 0.3, "proto_plate");
    p.session_transformation = Xform::translation(1.0, 2.0, 3.0);

    std::string path = "serialization/test_plate_element.bin";
    p.pb_dump(path);
    PlateElement loaded = PlateElement::pb_load(path);

    MINI_CHECK(loaded.name == "proto_plate");
    MINI_CHECK(TOLERANCE.is_close(loaded.thickness(), 0.3));
    MINI_CHECK(loaded.polygon().size() == 4);
    MINI_CHECK(TOLERANCE.is_close(loaded.polygon()[1][0], 2.0));
}

MINI_TEST("PlateElement", "From Top Bottom") {
    // uncomment #include "element_plate.h"
    // uncomment #include "point.h"
    std::vector<Point> bottom = {Point(0,0,0), Point(2,0,0), Point(2,2,0), Point(0,2,0), Point(0,0,0)};
    std::vector<Point> top    = {Point(0,0,1), Point(2,0,1), Point(2,2,1), Point(0,2,1), Point(0,0,1)};
    PlateElement p(bottom, top, "tb_plate");
    MINI_CHECK(p.polygon().size() == 4);
    MINI_CHECK(p.polygon_top().size() == 4);
    MINI_CHECK(TOLERANCE.is_close(p.thickness(), 1.0));
    MINI_CHECK(TOLERANCE.is_close(p.polygon()[0][2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(p.polygon_top()[0][2], 1.0));
    // Reversed argument order should auto-swap
    PlateElement pr(top, bottom, "tb_plate_r");
    MINI_CHECK(TOLERANCE.is_close(pr.polygon()[0][2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pr.polygon_top()[0][2], 1.0));
}

///////////////////////////////////////////////////////////////////////////////////////////
// Element - Polylines
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("Element", "Polylines") {
    Mesh m = Mesh::from_vertices_and_faces(
        {Point(0,0,0), Point(1,0,0), Point(1,1,0), Point(0,1,0)},
        {{0, 1, 2, 3}});
    Element e(m, "test_element");

    MINI_CHECK(e.polylines().empty());
    MINI_CHECK(e.planes().empty());
    MINI_CHECK(e.edge_vectors().empty());
    MINI_CHECK(!e.axis().has_value());
}

///////////////////////////////////////////////////////////////////////////////////////////
// BeamElement - Polylines/Planes/Edge Vectors/Axis
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("BeamElement", "Polylines") {
    BeamElement b(0.1, 0.2, 3.0);
    auto pls = b.polylines();
    MINI_CHECK(pls.size() == 6);
    for (const auto& pl : pls) MINI_CHECK(pl.point_count() == 5);
}

MINI_TEST("BeamElement", "Planes") {
    BeamElement b(0.1, 0.2, 3.0);
    auto pls = b.planes();
    MINI_CHECK(pls.size() == 6);
    MINI_CHECK(TOLERANCE.is_close(pls[0].z_axis()[2], -1.0));
    MINI_CHECK(TOLERANCE.is_close(pls[1].z_axis()[2], 1.0));
}

MINI_TEST("BeamElement", "Edge Vectors") {
    BeamElement b(0.1, 0.2, 3.0);
    auto evs = b.edge_vectors();
    MINI_CHECK(evs.size() == 12);
}

MINI_TEST("BeamElement", "Axis") {
    BeamElement b(0.1, 0.2, 5.0);
    auto ax = b.axis();
    MINI_CHECK(ax.has_value());
    MINI_CHECK(TOLERANCE.is_close(ax->start()[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(ax->end()[2], 5.0));
}

///////////////////////////////////////////////////////////////////////////////////////////
// ColumnElement - Polylines/Planes/Edge Vectors/Axis
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("ColumnElement", "Polylines") {
    ColumnElement c(0.4, 0.4, 3.0);
    auto pls = c.polylines();
    MINI_CHECK(pls.size() == 6);
    for (const auto& pl : pls) MINI_CHECK(pl.point_count() == 5);
}

MINI_TEST("ColumnElement", "Planes") {
    ColumnElement c(0.4, 0.4, 3.0);
    auto pls = c.planes();
    MINI_CHECK(pls.size() == 6);
    MINI_CHECK(TOLERANCE.is_close(pls[0].z_axis()[2], -1.0));
    MINI_CHECK(TOLERANCE.is_close(pls[1].z_axis()[2], 1.0));
}

MINI_TEST("ColumnElement", "Edge Vectors") {
    ColumnElement c(0.4, 0.4, 3.0);
    auto evs = c.edge_vectors();
    MINI_CHECK(evs.size() == 12);
}

MINI_TEST("ColumnElement", "Axis") {
    ColumnElement c(0.4, 0.4, 5.0);
    auto ax = c.axis();
    MINI_CHECK(ax.has_value());
    MINI_CHECK(TOLERANCE.is_close(ax->start()[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(ax->end()[2], 5.0));
}

///////////////////////////////////////////////////////////////////////////////////////////
// PlateElement - Polylines/Planes/Edge Vectors/Axis/Joinery
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("PlateElement", "Polylines") {
    std::vector<Point> polygon = {Point(0,0,0), Point(1,0,0), Point(1,1,0), Point(0,1,0)};
    PlateElement p(polygon, 0.2);
    auto pls = p.polylines();
    MINI_CHECK(pls.size() == 6);
    MINI_CHECK(pls[0].point_count() == 5);
    MINI_CHECK(pls[1].point_count() == 5);
    for (size_t i = 2; i < 6; ++i) MINI_CHECK(pls[i].point_count() == 5);
}

MINI_TEST("PlateElement", "Planes") {
    std::vector<Point> polygon = {Point(0,0,0), Point(1,0,0), Point(1,1,0), Point(0,1,0)};
    PlateElement p(polygon, 0.2);
    auto pls = p.planes();
    MINI_CHECK(pls.size() == 6);
    MINI_CHECK(TOLERANCE.is_close(pls[0].z_axis()[2], 1.0));
    MINI_CHECK(TOLERANCE.is_close(pls[1].z_axis()[2], -1.0));
}

MINI_TEST("PlateElement", "Edge Vectors") {
    std::vector<Point> polygon = {Point(0,0,0), Point(1,0,0), Point(1,1,0), Point(0,1,0)};
    PlateElement p(polygon, 0.2);
    auto evs = p.edge_vectors();
    MINI_CHECK(evs.size() == 4);
    MINI_CHECK(TOLERANCE.is_close(evs[0][0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(evs[0][1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(evs[1][0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(evs[1][1], 1.0));
}

MINI_TEST("PlateElement", "Axis") {
    std::vector<Point> polygon = {Point(0,0,0), Point(2,0,0), Point(2,2,0), Point(0,2,0)};
    PlateElement p(polygon, 0.4);
    auto ax = p.axis();
    MINI_CHECK(ax.has_value());
    MINI_CHECK(TOLERANCE.is_close(ax->start()[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(ax->start()[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(ax->start()[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(ax->end()[2], -0.4));
}

MINI_TEST("PlateElement", "Joint Types") {
    PlateElement p;
    MINI_CHECK(p.joint_types().empty());
    p.set_joint_types({1, 2, 3, 4});
    MINI_CHECK(p.joint_types().size() == 4);
    MINI_CHECK(p.joint_types()[0] == 1);
    MINI_CHECK(p.joint_types()[3] == 4);
}

MINI_TEST("PlateElement", "J Mf") {
    PlateElement p;
    MINI_CHECK(p.j_mf().empty());
    p.set_j_mf({
        {{0, true, 0.5}, {1, false, 0.3}},
        {},
        {{2, true, 0.8}},
    });
    MINI_CHECK(p.j_mf().size() == 3);
    MINI_CHECK(p.j_mf()[0].size() == 2);
    MINI_CHECK(p.j_mf()[0][0].joint_id == 0);
    MINI_CHECK(p.j_mf()[0][0].is_male == true);
    MINI_CHECK(TOLERANCE.is_close(p.j_mf()[0][0].parameter, 0.5));
    MINI_CHECK(p.j_mf()[2][0].joint_id == 2);
}

MINI_TEST("PlateElement", "Key") {
    PlateElement p;
    MINI_CHECK(p.key().empty());
    p.set_key("plate_A");
    MINI_CHECK(p.key() == "plate_A");
}

MINI_TEST("PlateElement", "Component Plane") {
    PlateElement p;
    MINI_CHECK(!p.component_plane().has_value());
    p.set_component_plane(Plane(Point(1, 2, 3), Vector(1, 0, 0), Vector(0, 1, 0)));
    MINI_CHECK(TOLERANCE.is_close(p.component_plane()->origin()[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(p.component_plane()->origin()[1], 2.0));
}

MINI_TEST("PlateElement", "Json Roundtrip Joinery") {
    std::vector<Point> polygon = {Point(0,0,0), Point(2,0,0), Point(2,2,0), Point(0,2,0)};
    PlateElement p(polygon, 0.3, "joinery_plate");
    p.set_joint_types({1, 2, 3, 4});
    p.set_j_mf({{{0, true, 0.5}}, {}, {{1, false, 0.3}}});
    p.set_key("plate_A");
    p.set_component_plane(Plane(Point(1, 2, 3), Vector(1, 0, 0), Vector(0, 1, 0)));

    std::string fname = "serialization/test_plate_element_joinery.json";
    p.json_dump(fname);
    PlateElement loaded = PlateElement::json_load(fname);

    MINI_CHECK(loaded.joint_types().size() == 4);
    MINI_CHECK(loaded.j_mf().size() == 3);
    MINI_CHECK(loaded.key() == "plate_A");
    MINI_CHECK(loaded.component_plane().has_value());
    MINI_CHECK(TOLERANCE.is_close(loaded.component_plane()->origin()[0], 1.0));
}

MINI_TEST("PlateElement", "Protobuf Roundtrip Joinery") {
    std::vector<Point> polygon = {Point(0,0,0), Point(2,0,0), Point(2,2,0), Point(0,2,0)};
    PlateElement p(polygon, 0.3, "joinery_plate");
    p.set_joint_types({1, 2, 3, 4});
    p.set_j_mf({{{0, true, 0.5}}, {}, {{1, false, 0.3}}});
    p.set_key("plate_A");
    p.set_component_plane(Plane(Point(1, 2, 3), Vector(1, 0, 0), Vector(0, 1, 0)));

    std::string path = "serialization/test_plate_element_joinery.bin";
    p.pb_dump(path);
    PlateElement loaded = PlateElement::pb_load(path);

    MINI_CHECK(loaded.joint_types().size() == 4);
    MINI_CHECK(loaded.j_mf().size() == 3);
    MINI_CHECK(loaded.j_mf()[0][0].joint_id == 0);
    MINI_CHECK(loaded.j_mf()[0][0].is_male == true);
    MINI_CHECK(loaded.key() == "plate_A");
    MINI_CHECK(loaded.component_plane().has_value());
    MINI_CHECK(TOLERANCE.is_close(loaded.component_plane()->origin()[0], 1.0));
}

} // namespace session_cpp
