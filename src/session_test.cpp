#include "mini_test.h"
#include "session.h"
#include "file_encoders.h"
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

MINI_TEST("Session", "Select By Type") {
    // uncomment #include "session.h"
    // uncomment #include "polyline.h"

    Session session;
    auto g0 = session.add_group("g0");
    auto g1 = session.add_group("g1");
    auto g2 = session.add_group("g2");

    session.add_polyline(std::make_shared<Polyline>(std::vector<Point>{Point(0,0,0), Point(1,0,0)}), g0);
    session.add_polyline(std::make_shared<Polyline>(std::vector<Point>{Point(0,1,0), Point(1,1,0)}), g0);
    session.add_polyline(std::make_shared<Polyline>(std::vector<Point>{Point(0,2,0), Point(1,2,0)}), g1);
    session.add_point(std::make_shared<Point>(9, 9, 9), g2);

    std::vector<std::vector<Polyline>> groups = session.select_by_type<Polyline>();

    MINI_CHECK(groups.size() == 2);
    MINI_CHECK(groups[0].size() == 2);
    MINI_CHECK(groups[1].size() == 1);
    MINI_CHECK(TOLERANCE.is_close(groups[1][0].get_point(0)[1], 2.0));

    MINI_CHECK(session.select_by_type<Mesh>().empty());
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

    Session session;
    auto plate = std::make_shared<Element>("p1");
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

    auto placed = std::make_shared<Mesh>();
    placed->add_vertex(Point(-1.0, -1.0, 0.0), 0);
    placed->add_vertex(Point(1.0, -1.0, 0.0), 1);
    placed->add_vertex(Point(0.0, 1.0, 0.0), 2);
    placed->add_face(std::vector<size_t>{0, 1, 2});
    std::string placed_guid = placed->guid();
    session.add_mesh(placed);
    session.set_xform(placed_guid, Xform::translation(100.0, 0.0, 0.0));
    auto hits2 = session.ray_cast(Point(100.0, 0.0, 2.0), Vector(0.0, 0.0, -1.0));

    MINI_CHECK(hits2.size() >= 1);
    MINI_CHECK(TOLERANCE.is_close(hits2[0].hit_point[0], 100.0));
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
    // uncomment #include "element.h"

    Session session;
    auto point = std::make_shared<Point>(1.0, 2.0, 3.0);
    session.add_point(point);
    bool removed = session.remove_object(point->guid());

    auto plate = std::make_shared<Element>("p1");
    std::string eguid = plate->guid();
    session.add_element(plate);
    bool eremoved = session.remove_object(eguid);

    std::string fname = "serialization/test_session_remove.bin";
    session.pb_dump(fname);
    Session loaded = Session::pb_load(fname);

    MINI_CHECK(removed);
    MINI_CHECK(session.lookup.count(point->guid()) == 0);
    MINI_CHECK(eremoved);
    MINI_CHECK(session.objects.elements->size() == 0);
    MINI_CHECK(loaded.lookup.count(eguid) == 0); // removed objects must not resurrect on save/load
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

MINI_TEST("Session", "Get Geometry Is Pure") {
    // get_geometry() is const: it returns a flattened SNAPSHOT and must never touch the
    // session's own geometry, so calling it twice gives the same answer.
    // uncomment #include "session.h"
    // uncomment #include "point.h"
    // uncomment #include "xform.h"

    Session session;
    auto point = std::make_shared<Point>(1.0, 2.0, 3.0);
    std::string guid = point->guid();
    session.add_point(point);
    session.set_xform(guid, Xform::translation(10.0, 0.0, 0.0));

    Point first = *session.get_geometry().points->at(0);
    Point second = *session.get_geometry().points->at(0);

    MINI_CHECK(TOLERANCE.is_close(first[0], 11.0));
    MINI_CHECK(TOLERANCE.is_close(second[0], 11.0));
    MINI_CHECK(TOLERANCE.is_close((*point)[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close((*session.objects.points->at(0))[0], 1.0));
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
    //   file_json_dumps()    │ std::string  │ to JSON string
    //   file_json_loads(s)   │ std::string  │ from JSON string
    //   file_json_dump(path) │ file         │ write to file
    //   file_json_load(path) │ file         │ read from file

    std::string fname = "serialization/test_session.json";
    session.file_json_dump(fname);
    Session loaded = Session::file_json_load(fname);

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

MINI_TEST("Session", "Lookup Mutation Roundtrip") {
    // uncomment #include "session.h"
    // uncomment #include "line.h"

    Session session;
    auto line = std::make_shared<Line>(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    std::string guid = line->guid();
    session.add_line(line);

    std::get<std::shared_ptr<Line>>(session.lookup[guid])->width = 5.0;

    std::string fname = "serialization/test_session_lookup.bin";
    session.pb_dump(fname);
    Session loaded = Session::pb_load(fname);

    MINI_CHECK(loaded.objects.lines->at(0)->width == 5.0);
    MINI_CHECK(std::get<std::shared_ptr<Line>>(loaded.lookup[guid])->width == 5.0);
}

MINI_TEST("Session", "Order") {
    // uncomment #include "session.h"
    // uncomment #include "line.h"
    // uncomment #include "point.h"

    Session session;
    auto line = std::make_shared<Line>(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    auto point = std::make_shared<Point>(1.0, 2.0, 3.0);
    std::string line_guid = line->guid();
    std::string point_guid = point->guid();
    session.add_line(line);
    session.add_point(point);

    std::vector<std::string> order = session.order();

    std::string fname = "serialization/test_session_order.bin";
    session.pb_dump(fname);
    Session loaded = Session::pb_load(fname);

    MINI_CHECK(order.size() == 2);
    MINI_CHECK(order[0] == point_guid);
    MINI_CHECK(order[1] == line_guid);
    MINI_CHECK(loaded.order() == order);
}

MINI_TEST("Session", "Set Xform") {
    // uncomment #include "session.h"
    // uncomment #include "point.h"
    // uncomment #include "xform.h"

    Session session;
    auto point = std::make_shared<Point>(1.0, 2.0, 3.0);
    std::string guid = point->guid();
    session.add_point(point);

    Xform shift = Xform::translation(5.0, 0.0, 0.0);
    session.set_xform(guid, shift);

    MINI_CHECK(session.xform(guid) == shift);
    // No parent was passed, so the object has no tree node: it is its own root and keeps
    // its placement. Falling back to identity here would move it to the origin.
    MINI_CHECK(session.world_xform(guid) == shift);
    MINI_CHECK(session.world_xforms()[guid] == shift);
    MINI_CHECK(session.xform("missing") == Xform::identity());
    MINI_CHECK(session.remove_xform(guid));
    MINI_CHECK(session.xform(guid) == Xform::identity());
}

MINI_TEST("Session", "World Xform Hierarchy") {
    // uncomment #include "session.h"
    // uncomment #include "point.h"
    // uncomment #include "xform.h"

    Session session;
    auto a = std::make_shared<Point>(0.0, 0.0, 0.0);
    auto b = std::make_shared<Point>(0.0, 0.0, 0.0);
    auto c = std::make_shared<Point>(0.0, 0.0, 0.0);
    std::string a_guid = a->guid();
    std::string b_guid = b->guid();
    std::string c_guid = c->guid();
    auto a_node = session.add_point(a);
    auto b_node = session.add_point(b);
    auto c_node = session.add_point(c);

    session.add(a_node);
    session.add(b_node, a_node);
    session.add(c_node, b_node);

    // Rotation and translation do not commute, so a reversed fold fails these checks.
    Xform a_xform = Xform::rotation_z(Tolerance::PI / 2.0);
    Xform b_xform = Xform::translation(2.0, 0.0, 0.0);
    Xform c_xform = Xform::rotation_z(Tolerance::PI / 2.0);
    session.set_xform(a_guid, a_xform);
    session.set_xform(b_guid, b_xform);
    session.set_xform(c_guid, c_xform);

    auto world = session.world_xforms();

    MINI_CHECK(session.world_xform(a_guid) == a_xform);
    MINI_CHECK(session.world_xform(b_guid) == a_xform * b_xform);
    MINI_CHECK(session.world_xform(c_guid) == a_xform * b_xform * c_xform);
    MINI_CHECK(world[c_guid] == session.world_xform(c_guid));
}

MINI_TEST("Session", "Xform Roundtrip") {
    // uncomment #include "session.h"
    // uncomment #include "point.h"
    // uncomment #include "xform.h"

    Session session;
    auto point = std::make_shared<Point>(1.0, 2.0, 3.0);
    std::string guid = point->guid();
    session.add_point(point);
    session.set_xform(guid, Xform::translation(7.0, 8.0, 9.0));

    std::string fname = "serialization/test_session_xform.bin";
    session.pb_dump(fname);
    Session loaded = Session::pb_load(fname);
    Session json_loaded = Session::file_json_loads(session.file_json_dumps());

    MINI_CHECK(loaded.xform(guid) == session.xform(guid));
    MINI_CHECK(loaded.xforms.size() == 1);
    MINI_CHECK(json_loaded.xform(guid) == session.xform(guid));
    MINI_CHECK(json_loaded.xforms.size() == 1);
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
    std::string box1_guid = box1->guid();
    auto box1_node = scene.add_mesh(box1);
    auto box2 = create_box(Point(0, 0, 0), 2.0);
    std::string box2_guid = box2->guid();
    auto box2_node = scene.add_mesh(box2);
    auto box3 = create_box(Point(0, 0, 0), 2.0);
    std::string box3_guid = box3->guid();
    auto box3_node = scene.add_mesh(box3);

    scene.add(box1_node);
    scene.add(box2_node, box1_node);
    scene.add(box3_node, box2_node);

    Plane plane_from(Point(0,0,0), Vector(1,0,0), Vector(0,1,0));
    Plane plane_to(Point(0,0,1.0), Vector(1,0,0), Vector(0,1,0));
    Xform xy_to_top = Xform::plane_to_plane(plane_from, plane_to);
    scene.set_xform(box1_guid, Xform::rotation_z(Tolerance::PI / 1.5) * xy_to_top);
    scene.set_xform(box2_guid, Xform::translation(2.0, 0, 0) * Xform::rotation_z(Tolerance::PI / 6.0));
    scene.set_xform(box3_guid, Xform::translation(2.0, 0, 0));

    // get_geometry BAKES the cumulative placement into the coordinates, so the deepest box
    // must land exactly where its world xform sends the original corner.
    Xform world3 = scene.world_xform(box3_guid);
    Point expected = world3.transform_point(Point(-1.0, -1.0, -1.0));
    Objects transformed = scene.get_geometry();
    Point baked = *(*transformed.meshes)[2]->vertex_point(0);

    MINI_CHECK(transformed.meshes->size() == 3);
    MINI_CHECK(TOLERANCE.is_close(baked[0], expected[0]));
    MINI_CHECK(TOLERANCE.is_close(baked[1], expected[1]));
    MINI_CHECK(TOLERANCE.is_close(baked[2], expected[2]));
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
    // uncomment #include "file_encoders.h"

    Session original;
    Component c;
    c.type_name = "FloorBuilder";
    c.name      = "floor_builder";
    c.extra     = {{"size", 3000}, {"height", 650}, {"rise", 453}};
    std::string guid = c.guid();
    original.add_component(c);

    std::string filename = "serialization/test_session_component.json";
    file_encoders::file_json_dump(original, filename);
    Session loaded = file_encoders::file_json_load<Session>(filename);

    MINI_CHECK(loaded.objects.components->size() == 1);
    MINI_CHECK(loaded.objects.components->at(0).type_name     == "FloorBuilder");
    MINI_CHECK(loaded.objects.components->at(0).extra["size"] == 3000);
    MINI_CHECK(loaded.objects.components->at(0).guid()        == guid);
}

} // namespace session_cpp
