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

MINI_TEST("Element", "AABB") {
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
    e.file_json_dump(fname);
    Element loaded = Element::file_json_load(fname);

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

} // namespace session_cpp
