#include "mini_test.h"
#include "session.h"
#include "encoders.h"
#include "tolerance.h"
#include <filesystem>
#include <fstream>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Session", "Constructor") {
    // uncomment #include "session.h"

    Session session;
    Session named("my_named_session");

    MINI_CHECK(session.name == "my_session");
    MINI_CHECK(!session.guid().empty());
    MINI_CHECK(named.name == "my_named_session");
}

MINI_TEST("Session", "Add Point") {
    // uncomment #include "session.h"
    // uncomment #include "point.h"

    Session session;
    auto point = std::make_shared<Point>(1.0, 2.0, 3.0);
    session.add_point(point);

    MINI_CHECK(session.objects.points->size() == 1);
    MINI_CHECK(session.lookup.count(point->guid()) == 1);
    MINI_CHECK(session.graph.has_node(point->guid()));
}

MINI_TEST("Session", "Add Line") {
    // uncomment #include "session.h"
    // uncomment #include "line.h"

    Session session;
    auto line = std::make_shared<Line>(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    session.add_line(line);

    MINI_CHECK(session.objects.lines->size() == 1);
    MINI_CHECK(session.lookup.count(line->guid()) == 1);
}

MINI_TEST("Session", "Add Plane") {
    // uncomment #include "session.h"
    // uncomment #include "plane.h"

    Session session;
    auto plane = std::make_shared<Plane>(Plane::xy_plane());
    session.add_plane(plane);

    MINI_CHECK(session.objects.planes->size() == 1);
    MINI_CHECK(session.lookup.count(plane->guid()) == 1);
}

MINI_TEST("Session", "Add OBB") {
    // uncomment #include "session.h"
    // uncomment #include "obb.h"

    Session session;
    auto obb = std::make_shared<OBB>(
        Point(0.0, 0.0, 0.0),
        Vector(1.0, 0.0, 0.0),
        Vector(0.0, 1.0, 0.0),
        Vector(0.0, 0.0, 1.0),
        Vector(1.0, 1.0, 1.0)
    );
    session.add_obb(obb);

    MINI_CHECK(session.objects.bboxes->size() == 1);
    MINI_CHECK(session.lookup.count(obb->guid()) == 1);
}

MINI_TEST("Session", "Add Polyline") {
    // uncomment #include "session.h"
    // uncomment #include "polyline.h"

    Session session;
    auto pl = std::make_shared<Polyline>(std::vector<Point>{Point(0,0,0), Point(1,0,0), Point(1,1,0)});
    session.add_polyline(pl);

    MINI_CHECK(session.objects.polylines->size() == 1);
    MINI_CHECK(session.lookup.count(pl->guid()) == 1);
}

MINI_TEST("Session", "Add Pointcloud") {
    // uncomment #include "session.h"
    // uncomment #include "pointcloud.h"

    Session session;
    auto pc = std::make_shared<PointCloud>(std::vector<Point>{Point(0,0,0), Point(1,0,0)}, std::vector<Vector>{}, std::vector<Color>{});
    session.add_pointcloud(pc);

    MINI_CHECK(session.objects.pointclouds->size() == 1);
    MINI_CHECK(session.lookup.count(pc->guid()) == 1);
}

MINI_TEST("Session", "Add Mesh") {
    // uncomment #include "session.h"
    // uncomment #include "mesh.h"

    Session session;
    auto mesh = std::make_shared<Mesh>();
    mesh->add_vertex(Point(0,0,0), 0);
    mesh->add_vertex(Point(1,0,0), 1);
    mesh->add_vertex(Point(0,1,0), 2);
    mesh->add_face(std::vector<size_t>{0, 1, 2});
    session.add_mesh(mesh);

    MINI_CHECK(session.objects.meshes->size() == 1);
    MINI_CHECK(session.lookup.count(mesh->guid()) == 1);
}

MINI_TEST("Session", "Add Nurbscurve") {
    // uncomment #include "session.h"
    // uncomment #include "nurbscurve.h"

    Session session;
    std::vector<Point> pts = {Point(0,0,0), Point(1,1,0), Point(2,0,0), Point(3,1,0)};
    auto nc = std::make_shared<NurbsCurve>(NurbsCurve::create(false, 2, pts));
    session.add_nurbscurve(nc);

    MINI_CHECK(session.objects.nurbscurves->size() == 1);
    MINI_CHECK(session.lookup.count(nc->guid()) == 1);
}

MINI_TEST("Session", "Add Nurbssurface") {
    // uncomment #include "session.h"
    // uncomment #include "nurbssurface.h"

    Session session;
    std::vector<Point> pts = {
        Point(0,0,0), Point(0,1,0), Point(0,2,0), Point(0,3,0),
        Point(1,0,0), Point(1,1,0), Point(1,2,0), Point(1,3,0),
        Point(2,0,0), Point(2,1,0), Point(2,2,0), Point(2,3,0),
        Point(3,0,0), Point(3,1,0), Point(3,2,0), Point(3,3,0),
    };
    auto ns = std::make_shared<NurbsSurface>(NurbsSurface::create(false, false, 3, 3, 4, 4, pts));
    session.add_nurbssurface(ns);

    MINI_CHECK(session.objects.nurbssurfaces->size() == 1);
    MINI_CHECK(session.lookup.count(ns->guid()) == 1);
}

MINI_TEST("Session", "Add Brep") {
    // uncomment #include "session.h"
    // uncomment #include "brep.h"

    Session session;
    auto brep = std::make_shared<BRep>(BRep::create_box(1.0, 1.0, 1.0));
    session.add_brep(brep);

    MINI_CHECK(session.objects.breps->size() == 1);
    MINI_CHECK(session.lookup.count(brep->guid()) == 1);
}

MINI_TEST("Session", "Add Element") {
    // uncomment #include "session.h"
    // uncomment #include "element_plate.h"

    Session session;
    std::vector<Point> polygon = {Point(0,0,0), Point(2,0,0), Point(2,2,0), Point(0,2,0)};
    auto plate = std::make_shared<ElementPlate>(polygon, 0.2, "p1");
    session.add_element(plate);

    MINI_CHECK(session.objects.elements->size() == 1);
    MINI_CHECK(session.lookup.count(plate->guid()) == 1);
    MINI_CHECK(session.graph.has_node(plate->guid()));
}

MINI_TEST("Session", "Add Group") {
    // uncomment #include "session.h"

    Session session;
    auto group = session.add_group("my_group");

    MINI_CHECK(group != nullptr);
    MINI_CHECK(group->name == "my_group");
}

MINI_TEST("Session", "Add Edge") {
    // uncomment #include "session.h"
    // uncomment #include "point.h"

    Session session;
    auto p1 = std::make_shared<Point>(1.0, 2.0, 3.0);
    auto p2 = std::make_shared<Point>(4.0, 5.0, 6.0);
    session.add_point(p1);
    session.add_point(p2);
    session.add_edge(p1->guid(), p2->guid(), "connection");

    MINI_CHECK(session.graph.has_edge({p1->guid(), p2->guid()}));
}

MINI_TEST("Session", "Add Feature") {
    // uncomment #include "session.h"
    // uncomment #include "feature.h"
    // uncomment #include "point.h"

    Session session;
    auto p1 = std::make_shared<Point>(0.0, 0.0, 0.0);
    auto p2 = std::make_shared<Point>(1.0, 0.0, 0.0);
    session.add_point(p1);
    session.add_point(p2);
    FaceFeature f;
    f.face_id_a = 0;
    f.face_id_b = 0;
    std::string fguid = session.add_feature(p1->guid(), p2->guid(), EdgeFeature{f});

    MINI_CHECK(!fguid.empty());
    MINI_CHECK(session.edge_features.count(fguid) == 1);
    MINI_CHECK(session.graph.has_edge({p1->guid(), p2->guid()}));
}

MINI_TEST("Session", "Add Hierarchy") {
    // uncomment #include "session.h"
    // uncomment #include "point.h"

    Session session;
    auto p1 = std::make_shared<Point>(0,0,0);
    auto p2 = std::make_shared<Point>(1,0,0);
    auto n1 = session.add_point(p1);
    auto n2 = session.add_point(p2);
    session.add(n1);
    session.add(n2);
    bool ok = session.add_hierarchy(n1->guid(), n2->guid());

    MINI_CHECK(ok);
}

MINI_TEST("Session", "Get Children") {
    // uncomment #include "session.h"
    // uncomment #include "point.h"

    Session session;
    auto p1 = std::make_shared<Point>(0,0,0);
    auto p2 = std::make_shared<Point>(1,0,0);
    auto n1 = session.add_point(p1);
    auto n2 = session.add_point(p2);
    session.add(n1);
    session.add(n2);
    session.add_hierarchy(n1->guid(), n2->guid());

    auto children = session.get_children(n1->guid());

    MINI_CHECK(children.size() == 1);
    MINI_CHECK(children[0] == n2->guid());
}

MINI_TEST("Session", "Add Relationship") {
    // uncomment #include "session.h"
    // uncomment #include "point.h"

    Session session;
    auto p1 = std::make_shared<Point>(0,0,0);
    auto p2 = std::make_shared<Point>(1,0,0);
    session.add_point(p1);
    session.add_point(p2);
    session.add_relationship(p1->guid(), p2->guid(), "connects_to");

    MINI_CHECK(session.graph.has_edge({p1->guid(), p2->guid()}));
}

MINI_TEST("Session", "Get Neighbours") {
    // uncomment #include "session.h"
    // uncomment #include "point.h"

    Session session;
    auto p1 = std::make_shared<Point>(0,0,0);
    auto p2 = std::make_shared<Point>(1,0,0);
    session.add_point(p1);
    session.add_point(p2);
    session.add_edge(p1->guid(), p2->guid(), "connection");

    auto neighbours = session.get_neighbours(p1->guid());

    MINI_CHECK(neighbours.size() == 1);
    MINI_CHECK(neighbours[0] == p2->guid());
}

MINI_TEST("Session", "Get Collisions") {
    // uncomment #include "session.h"
    // uncomment #include "obb.h"

    Session session;
    auto obb1 = std::make_shared<OBB>(
        Point(0.0, 0.0, 0.0),
        Vector(1.0, 0.0, 0.0),
        Vector(0.0, 1.0, 0.0),
        Vector(0.0, 0.0, 1.0),
        Vector(2.0, 2.0, 2.0)
    );
    auto obb2 = std::make_shared<OBB>(
        Point(1.0, 0.0, 0.0),
        Vector(1.0, 0.0, 0.0),
        Vector(0.0, 1.0, 0.0),
        Vector(0.0, 0.0, 1.0),
        Vector(2.0, 2.0, 2.0)
    );
    session.add_obb(obb1);
    session.add_obb(obb2);
    auto pairs = session.get_collisions();

    MINI_CHECK(pairs.size() >= 1);
}

MINI_TEST("Session", "Ray Cast") {
    // uncomment #include "session.h"
    // uncomment #include "mesh.h"

    Session session;
    auto mesh = std::make_shared<Mesh>();
    mesh->add_vertex(Point(-1.0, -1.0, 0.0), 0);
    mesh->add_vertex(Point(1.0, -1.0, 0.0), 1);
    mesh->add_vertex(Point(0.0, 1.0, 0.0), 2);
    mesh->add_face(std::vector<size_t>{0, 1, 2});
    session.add_mesh(mesh);
    auto hits = session.ray_cast(Point(0.0, 0.0, 2.0), Vector(0.0, 0.0, -1.0));

    MINI_CHECK(hits.size() >= 1);
}

MINI_TEST("Session", "Get Object") {
    // uncomment #include "session.h"
    // uncomment #include "point.h"

    Session session;
    auto point = std::make_shared<Point>(1.0, 2.0, 3.0);
    session.add_point(point);

    auto retrieved = session.get_object<Point>(point->guid());

    MINI_CHECK(retrieved != nullptr);
    MINI_CHECK(retrieved->guid() == point->guid());
}

MINI_TEST("Session", "Remove Object") {
    // uncomment #include "session.h"
    // uncomment #include "point.h"

    Session session;
    auto point = std::make_shared<Point>(1.0, 2.0, 3.0);
    session.add_point(point);
    bool removed = session.remove_object(point->guid());

    MINI_CHECK(removed);
    MINI_CHECK(session.lookup.count(point->guid()) == 0);
}

MINI_TEST("Session", "Get Geometry") {
    // uncomment #include "session.h"
    // uncomment #include "point.h"

    Session session;
    auto point = std::make_shared<Point>(1.0, 2.0, 3.0);
    session.add_point(point);

    Objects geom = session.get_geometry();

    MINI_CHECK(geom.points->size() == 1);
}

MINI_TEST("Session", "Compute Face To Face") {
    // uncomment #include "session.h"
    // uncomment #include "element_plate.h"

    Session session;
    auto p1 = std::make_shared<ElementPlate>(std::vector<Point>{Point(0,0,0), Point(1,0,0), Point(1,1,0), Point(0,1,0)}, 0.2, "p1");
    auto p2 = std::make_shared<ElementPlate>(std::vector<Point>{Point(0,0,-0.2), Point(1,0,-0.2), Point(1,1,-0.2), Point(0,1,-0.2)}, 0.2, "p2");
    session.add_element(p1);
    session.add_element(p2);
    session.compute_face_to_face(5.0, 0.001);

    MINI_CHECK(session.objects.elements->size() == 2);
    MINI_CHECK(session.graph.has_edge({p1->guid(), p2->guid()}));
}

MINI_TEST("Session", "Json Roundtrip") {
    // uncomment #include "session.h"
    // uncomment #include "point.h"

    Session session;
    auto p1 = std::make_shared<Point>(1.0, 2.0, 3.0);
    auto p2 = std::make_shared<Point>(4.0, 5.0, 6.0);
    session.add_point(p1);
    session.add_point(p2);
    session.add_edge(p1->guid(), p2->guid(), "connection");

    //   jsondump()      │ ordered_json │ to JSON object (internal use)
    //   jsonload(j)     │ ordered_json │ from JSON object (internal use)
    //   json_dumps()    │ std::string  │ to JSON string
    //   json_loads(s)   │ std::string  │ from JSON string
    //   json_dump(path) │ file         │ write to file
    //   json_load(path) │ file         │ read from file

    std::string fname = "serialization/test_session.json";
    session.json_dump(fname);
    Session loaded = Session::json_load(fname);

    MINI_CHECK(loaded.name == session.name);
    MINI_CHECK(loaded.lookup.size() == session.lookup.size());
    MINI_CHECK(loaded.graph.number_of_vertices() == session.graph.number_of_vertices());
}

MINI_TEST("Session", "Protobuf Roundtrip") {
    // uncomment #include "session.h"
    // uncomment #include "point.h"

    Session session;
    auto p1 = std::make_shared<Point>(1.0, 2.0, 3.0);
    auto p2 = std::make_shared<Point>(4.0, 5.0, 6.0);
    session.add_point(p1);
    session.add_point(p2);
    session.add_edge(p1->guid(), p2->guid(), "connection");

    std::string fname = "serialization/test_session.bin";
    session.pb_dump(fname);
    Session loaded = Session::pb_load(fname);

    MINI_CHECK(loaded.name == session.name);
    MINI_CHECK(loaded.lookup.size() == session.lookup.size());
}

MINI_TEST("Session", "Tree Transformation Hierarchy") {
    // uncomment #include "session.h"
    // uncomment #include "mesh.h"
    // uncomment #include "plane.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // uncomment #include "xform.h"

    Session scene("tree_transformation_test");

    auto create_box = [](const Point& center, double size) -> std::shared_ptr<Mesh> {
        auto mesh = std::make_shared<Mesh>();
        double h = size * 0.5;
        std::vector<Point> verts = {
            Point(center[0] - h, center[1] - h, center[2] - h),
            Point(center[0] + h, center[1] - h, center[2] - h),
            Point(center[0] + h, center[1] + h, center[2] - h),
            Point(center[0] - h, center[1] + h, center[2] - h),
            Point(center[0] - h, center[1] - h, center[2] + h),
            Point(center[0] + h, center[1] - h, center[2] + h),
            Point(center[0] + h, center[1] + h, center[2] + h),
            Point(center[0] - h, center[1] + h, center[2] + h)
        };
        for (size_t i = 0; i < verts.size(); ++i) mesh->add_vertex(verts[i], i);
        std::vector<std::vector<size_t>> faces = {
            {0,1,2,3}, {4,7,6,5}, {0,4,5,1}, {2,6,7,3}, {0,3,7,4}, {1,5,6,2}
        };
        for (const auto& f : faces) mesh->add_face(f);
        return mesh;
    };

    auto box1 = create_box(Point(0, 0, 0), 2.0);
    auto box1_node = scene.add_mesh(box1);
    auto box2 = create_box(Point(0, 0, 0), 2.0);
    auto box2_node = scene.add_mesh(box2);
    auto box3 = create_box(Point(0, 0, 0), 2.0);
    auto box3_node = scene.add_mesh(box3);

    scene.add(box1_node);
    scene.add(box2_node, box1_node);
    scene.add(box3_node, box2_node);

    Plane plane_from(Point(0,0,0), Vector(1,0,0), Vector(0,1,0));
    Plane plane_to(Point(0,0,1.0), Vector(1,0,0), Vector(0,1,0));
    Xform xy_to_top = Xform::plane_to_plane(plane_from, plane_to);
    box1->xform = Xform::rotation_z(Tolerance::PI / 1.5) * xy_to_top;
    box2->xform = Xform::translation(2.0, 0, 0) * Xform::rotation_z(Tolerance::PI / 6.0);
    box3->xform = Xform::translation(2.0, 0, 0);

    Objects transformed = scene.get_geometry();

    MINI_CHECK(transformed.meshes->size() == 3);
    auto& m1 = (*transformed.meshes)[0];
    Point v0 = m1->vertex.at(0).position();
    MINI_CHECK(std::abs(v0[0] - 1.36603) < 1e-4);
    MINI_CHECK(std::abs(v0[1] - (-0.366025)) < 1e-4);
    MINI_CHECK(std::abs(v0[2] - 0.0) < 1e-4);
}

MINI_TEST("Session", "Add Component") {
    // add_component stores a custom domain object in the session.
    // It is indexed by guid in component_lookup and registered in the graph.
    // uncomment #include "session.h"

    Session session;

    Component c;
    c.type_name = "FloorBuilder";
    c.name      = "floor_builder";
    c.extra     = {{"size", 3000}, {"height", 650}};
    std::string guid = c.guid();

    session.add_component(c);

    MINI_CHECK(session.objects.components->size() == 1);
    MINI_CHECK(session.component_lookup.count(guid) == 1);
    MINI_CHECK(session.graph.has_node(guid));
}

MINI_TEST("Session", "Component Json Roundtrip") {
    // A session with a component round-trips through JSON:
    // the component survives with all custom fields intact.
    // uncomment #include "session.h"
    // uncomment #include "encoders.h"

    Session original;
    Component c;
    c.type_name = "FloorBuilder";
    c.name      = "floor_builder";
    c.extra     = {{"size", 3000}, {"height", 650}, {"rise", 453}};
    std::string guid = c.guid();
    original.add_component(c);

    std::string filename = "serialization/test_session_component.json";
    encoders::json_dump(original, filename);
    Session loaded = encoders::json_load<Session>(filename);

    MINI_CHECK(loaded.objects.components->size() == 1);
    MINI_CHECK(loaded.objects.components->at(0).type_name     == "FloorBuilder");
    MINI_CHECK(loaded.objects.components->at(0).extra["size"] == 3000);
    MINI_CHECK(loaded.objects.components->at(0).guid()        == guid);
}

} // namespace session_cpp
