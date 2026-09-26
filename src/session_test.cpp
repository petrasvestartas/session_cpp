#include "mini_test.h"
#include "session.h"
#include "file_encoders.h"
#include "session.pb.h"
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/util/message_differencer.h>
#include "tolerance.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <tuple>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Session", "Constructor") {

    Session session;
    Session named("my_named_session");

    MINI_CHECK(session.name == "my_session");
    MINI_CHECK(!session.guid().empty());
    MINI_CHECK(named.name == "my_named_session");
}

MINI_TEST("Session", "Copy") {

    Session session("original");
    std::shared_ptr<Point> point = std::make_shared<Point>(1.0, 2.0, 3.0);
    std::shared_ptr<Element> element = std::make_shared<Element>("plate");
    std::shared_ptr<TreeNode> group = session.add_group("Group");
    session.add_point(point, group);
    session.add_element(element, group);
    session.add_edge(point->guid(), element->guid(), "touching");
    session.set_xform(point->guid(), Xform::translation(1.0, 0.0, 0.0));
    const std::string guid = session.guid();

    Session copy = session;

    MINI_CHECK(copy.name == session.name);
    MINI_CHECK(copy.guid() == guid);
    MINI_CHECK(copy.history.depth() == 0);
    MINI_CHECK(copy.objects.points->size() == 1);
    MINI_CHECK(copy.objects.elements->size() == 1);
    MINI_CHECK(copy.lookup.size() == session.lookup.size());
    MINI_CHECK(copy.graph.number_of_edges() == 1);
    MINI_CHECK(copy.xforms.size() == 1);
    MINI_CHECK(copy.tree.root()->descendants().size() == session.tree.root()->descendants().size());

    MINI_CHECK(copy.objects.points != session.objects.points);
    MINI_CHECK(copy.objects.elements != session.objects.elements);
    MINI_CHECK(copy.tree.root() != session.tree.root());
    MINI_CHECK(copy.objects.points->at(0) != session.objects.points->at(0));
    MINI_CHECK(copy.objects.points->at(0)->guid() == point->guid());
    MINI_CHECK(copy.objects.elements->at(0)->guid() == element->guid());
    MINI_CHECK(copy.lookup.count(point->guid()) == 1);

    copy.objects.points->clear();
    const std::vector<std::shared_ptr<TreeNode>> copied_nodes = copy.tree.nodes();

    MINI_CHECK(copied_nodes.size() > 1);

    copy.tree.remove(copied_nodes[1]);

    MINI_CHECK(session.objects.points->size() == 1);
    MINI_CHECK(copy.objects.points->size() == 0);
    MINI_CHECK(!session.tree.root()->descendants().empty());
    MINI_CHECK(copy.tree.root()->descendants().empty());
}

MINI_TEST("Session", "Add Point") {

    Session session;
    std::shared_ptr<Point> point = std::make_shared<Point>(1.0, 2.0, 3.0);
    session.add_point(point);

    MINI_CHECK(session.objects.points->size() == 1);
    MINI_CHECK(session.lookup.count(point->guid()) == 1);
    MINI_CHECK(session.graph.has_node(point->guid()));
}

MINI_TEST("Session", "Add Line") {

    Session session;
    std::shared_ptr<Line> line = std::make_shared<Line>(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    session.add_line(line);

    MINI_CHECK(session.objects.lines->size() == 1);
    MINI_CHECK(session.lookup.count(line->guid()) == 1);
}

MINI_TEST("Session", "Add Plane") {

    Session session;
    std::shared_ptr<Plane> plane = std::make_shared<Plane>(Plane::xy_plane());
    session.add_plane(plane);

    MINI_CHECK(session.objects.planes->size() == 1);
    MINI_CHECK(session.lookup.count(plane->guid()) == 1);
}

MINI_TEST("Session", "Add OBB") {

    Session session;
    std::shared_ptr<OBB> obb = std::make_shared<OBB>(
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

    Session session;
    std::shared_ptr<Polyline> pl =
        std::make_shared<Polyline>(std::vector<Point>{Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0)});

    session.add_polyline(pl);

    MINI_CHECK(session.objects.polylines->size() == 1);
    MINI_CHECK(session.lookup.count(pl->guid()) == 1);
}

MINI_TEST("Session", "Select By Type") {

    Session session;
    std::shared_ptr<TreeNode> g0 = session.add_group("g0");
    std::shared_ptr<TreeNode> g1 = session.add_group("g1");
    std::shared_ptr<TreeNode> g2 = session.add_group("g2");

    session.add_polyline(std::make_shared<Polyline>(std::vector<Point>{Point(0, 0, 0), Point(1, 0, 0)}), g0);
    session.add_polyline(std::make_shared<Polyline>(std::vector<Point>{Point(0, 1, 0), Point(1, 1, 0)}), g0);
    session.add_polyline(std::make_shared<Polyline>(std::vector<Point>{Point(0, 2, 0), Point(1, 2, 0)}), g1);
    session.add_point(std::make_shared<Point>(9, 9, 9), g2);

    std::vector<std::vector<Polyline>> groups = session.select_by_type<Polyline>();

    MINI_CHECK(groups.size() == 2);
    MINI_CHECK(groups[0].size() == 2);
    MINI_CHECK(groups[1].size() == 1);
    MINI_CHECK(TOLERANCE.is_close(groups[1][0].get_point(0)[1], 2.0));

    MINI_CHECK(session.select_by_type<Mesh>().empty());
}

MINI_TEST("Session", "Add Pointcloud") {

    Session session;
    std::shared_ptr<PointCloud> pc = std::make_shared<PointCloud>(
        std::vector<Point>{Point(0, 0, 0), Point(1, 0, 0)},
        std::vector<Vector>{},
        std::vector<Color>{}
    );
    session.add_pointcloud(pc);

    MINI_CHECK(session.objects.pointclouds->size() == 1);
    MINI_CHECK(session.lookup.count(pc->guid()) == 1);
}

MINI_TEST("Session", "Add Mesh") {

    Session session;
    std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
    mesh->add_vertex(Point(0, 0, 0), 0);
    mesh->add_vertex(Point(1, 0, 0), 1);
    mesh->add_vertex(Point(0, 1, 0), 2);
    mesh->add_face(std::vector<size_t>{0, 1, 2});
    session.add_mesh(mesh);

    MINI_CHECK(session.objects.meshes->size() == 1);
    MINI_CHECK(session.lookup.count(mesh->guid()) == 1);
}

MINI_TEST("Session", "Add Nurbscurve") {

    Session session;
    std::vector<Point> pts = {Point(0, 0, 0), Point(1, 1, 0), Point(2, 0, 0), Point(3, 1, 0)};
    std::shared_ptr<NurbsCurve> nc = std::make_shared<NurbsCurve>(NurbsCurve::create(false, 2, pts));
    session.add_nurbscurve(nc);

    MINI_CHECK(session.objects.nurbscurves->size() == 1);
    MINI_CHECK(session.lookup.count(nc->guid()) == 1);
}

MINI_TEST("Session", "Add Nurbssurface") {

    Session session;
    std::vector<Point> pts = {
        Point(0, 0, 0),
        Point(0, 1, 0),
        Point(0, 2, 0),
        Point(0, 3, 0),
        Point(1, 0, 0),
        Point(1, 1, 0),
        Point(1, 2, 0),
        Point(1, 3, 0),
        Point(2, 0, 0),
        Point(2, 1, 0),
        Point(2, 2, 0),
        Point(2, 3, 0),
        Point(3, 0, 0),
        Point(3, 1, 0),
        Point(3, 2, 0),
        Point(3, 3, 0),
    };
    std::shared_ptr<NurbsSurface> ns =
        std::make_shared<NurbsSurface>(NurbsSurface::create(false, false, 3, 3, 4, 4, pts));

    session.add_nurbssurface(ns);

    MINI_CHECK(session.objects.nurbssurfaces->size() == 1);
    MINI_CHECK(session.lookup.count(ns->guid()) == 1);
}

MINI_TEST("Session", "Add Brep") {

    Session session;
    std::shared_ptr<BRep> brep = std::make_shared<BRep>(BRep::create_box(1.0, 1.0, 1.0));
    session.add_brep(brep);

    MINI_CHECK(session.objects.breps->size() == 1);
    MINI_CHECK(session.lookup.count(brep->guid()) == 1);
}

MINI_TEST("Session", "Add Element") {

    Session session;
    std::shared_ptr<Element> plate = std::make_shared<Element>("p1");
    session.add_element(plate);

    MINI_CHECK(session.objects.elements->size() == 1);
    MINI_CHECK(session.lookup.count(plate->guid()) == 1);
    MINI_CHECK(session.graph.has_node(plate->guid()));
}

MINI_TEST("Session", "Add Empty Geometry") {

    Session session;
    std::shared_ptr<TreeNode> group = session.add_group("empty");

    MINI_CHECK(session.add_point(nullptr, group) == nullptr);
    MINI_CHECK(session.add_polyline(std::make_shared<Polyline>(std::vector<Point>{Point(0, 0, 0)}), group) == nullptr);
    MINI_CHECK(session.add_pointcloud(std::make_shared<PointCloud>(), group) == nullptr);
    MINI_CHECK(session.add_mesh(std::make_shared<Mesh>(), group) == nullptr);
    MINI_CHECK(session.add_nurbscurve(std::make_shared<NurbsCurve>(), group) == nullptr);
    MINI_CHECK(session.add_nurbssurface(std::make_shared<NurbsSurface>(), group) == nullptr);
    MINI_CHECK(session.add_brep(std::make_shared<BRep>(), group) == nullptr);

    std::shared_ptr<Mesh> vertices_only = std::make_shared<Mesh>();
    vertices_only->add_vertex(Point(0, 0, 0), 0);

    MINI_CHECK(session.add_mesh(vertices_only, group) == nullptr);

    session.add(session.add_mesh(std::make_shared<Mesh>(), group), group);

    MINI_CHECK(session.lookup.empty());
    MINI_CHECK(session.order().empty());
    MINI_CHECK(group->children().empty());
}

MINI_TEST("Session", "Add Group") {

    Session session;
    std::shared_ptr<TreeNode> group = session.add_group("my_group");

    MINI_CHECK(group != nullptr);
    MINI_CHECK(group->name == "my_group");
}

MINI_TEST("Session", "Add Edge") {

    Session session;
    std::shared_ptr<Point> p1 = std::make_shared<Point>(1.0, 2.0, 3.0);
    std::shared_ptr<Point> p2 = std::make_shared<Point>(4.0, 5.0, 6.0);
    session.add_point(p1);
    session.add_point(p2);
    session.add_edge(p1->guid(), p2->guid(), "connection");

    MINI_CHECK(session.graph.has_edge({p1->guid(), p2->guid()}));
}

MINI_TEST("Session", "Add Hierarchy") {

    Session session;
    std::shared_ptr<Point> p1 = std::make_shared<Point>(0, 0, 0);
    std::shared_ptr<Point> p2 = std::make_shared<Point>(1, 0, 0);
    std::shared_ptr<TreeNode> n1 = session.add_point(p1);
    std::shared_ptr<TreeNode> n2 = session.add_point(p2);
    session.add(n1);
    session.add(n2);
    bool ok = session.add_hierarchy(n1->guid(), n2->guid());

    MINI_CHECK(ok);
}

MINI_TEST("Session", "Get Children") {

    Session session;
    std::shared_ptr<Point> p1 = std::make_shared<Point>(0, 0, 0);
    std::shared_ptr<Point> p2 = std::make_shared<Point>(1, 0, 0);
    std::shared_ptr<TreeNode> n1 = session.add_point(p1);
    std::shared_ptr<TreeNode> n2 = session.add_point(p2);
    session.add(n1);
    session.add(n2);
    session.add_hierarchy(n1->guid(), n2->guid());

    std::vector<std::string> children = session.get_children(n1->guid());

    MINI_CHECK(children.size() == 1);
    MINI_CHECK(children[0] == n2->guid());
}

MINI_TEST("Session", "Add Relationship") {

    Session session;
    std::shared_ptr<Point> p1 = std::make_shared<Point>(0, 0, 0);
    std::shared_ptr<Point> p2 = std::make_shared<Point>(1, 0, 0);
    session.add_point(p1);
    session.add_point(p2);
    session.add_relationship(p1->guid(), p2->guid(), "connects_to");

    MINI_CHECK(session.graph.has_edge({p1->guid(), p2->guid()}));
}

namespace {

/// A test-only subclass: a named interaction with no state of its own.
class NamedInteraction : public Interaction {
public:
    /// Construct from a name.
    NamedInteraction(const std::string& name = "")
        : Interaction(name) {}

    /// Return the registered type name.
    std::string interaction_type_name() const override {
        return "NamedInteraction";
    }

    /// Return no state.
    std::string interaction_data_dumps() const override {
        return "";
    }

    /// Return a copy with the same guid.
    std::shared_ptr<Interaction> clone() const override {
        return std::make_shared<NamedInteraction>(*this);
    }
};

/// Build a NamedInteraction from its data.
std::shared_ptr<Interaction> named_interaction(const std::string&) {
    return std::make_shared<NamedInteraction>();
}

} // namespace

MINI_TEST("Session", "Add Interaction") {

    Session session;
    const std::shared_ptr<Element> a = std::make_shared<Element>("a");
    const std::shared_ptr<Element> b = std::make_shared<Element>("b");
    const std::shared_ptr<Element> absent = std::make_shared<Element>("absent");
    session.add_element(a);
    session.add_element(b);
    session.add_edge(a->guid(), b->guid(), "authored");
    const std::shared_ptr<Interaction> glue = session.add_interaction(a, b, std::make_shared<NamedInteraction>("glue"));
    const std::string id = session.graph.edges.at(a->guid()).at(b->guid()).guid();
    const std::shared_ptr<Interaction> screw = session.add_interaction(b, a, std::make_shared<NamedInteraction>("screw"));

    MINI_CHECK(session.interactions.size() == 1);
    MINI_CHECK(session.interactions.at(id).size() == 2);
    MINI_CHECK(session.interactions.at(id)[0]->guid() == glue->guid());
    MINI_CHECK(session.interactions.at(id)[1]->guid() == screw->guid());
    MINI_CHECK(session.graph.number_of_edges() == 1);
    MINI_CHECK(session.graph.edges.at(b->guid()).at(a->guid()).guid() == id);
    MINI_CHECK(session.graph.edges.at(a->guid()).at(b->guid()).attribute == "authored");

    bool missing_rejected = false;
    bool self_rejected = false;

    try {
        session.add_interaction(a, absent, std::make_shared<NamedInteraction>());
    } catch (const std::invalid_argument&) {
        missing_rejected = true;
    }

    try {
        session.add_interaction(a, a, std::make_shared<NamedInteraction>());
    } catch (const std::invalid_argument&) {
        self_rejected = true;
    }

    MINI_CHECK(missing_rejected);
    MINI_CHECK(self_rejected);
    MINI_CHECK(session.graph.number_of_edges() == 1);
}

MINI_TEST("Session", "Get Interaction") {

    Interaction::register_type("NamedInteraction", named_interaction);
    Session session;
    const std::shared_ptr<Element> a = std::make_shared<Element>("a");
    const std::shared_ptr<Element> b = std::make_shared<Element>("b");
    const std::shared_ptr<Element> c = std::make_shared<Element>("c");
    session.add_element(a);
    session.add_element(b);
    session.add_element(c);
    session.add_edge(a->guid(), c->guid(), "authored");
    const std::vector<std::shared_ptr<Interaction>> before = session.get_interaction(a, b);
    const std::vector<std::shared_ptr<Interaction>> bare = session.get_interaction(a, c);
    const std::shared_ptr<Interaction> glue = session.add_interaction(a, b, std::make_shared<NamedInteraction>("glue"));
    const std::string id = session.graph.edges.at(a->guid()).at(b->guid()).guid();
    Session duplicate = session;
    duplicate.interactions.at(id)[0]->name = "screw";
    const Session loaded_b = Session::pb_loads(session.pb_dumps());
    const Session loaded_j = Session::file_json_loads(session.file_json_dumps());

    MINI_CHECK(before.empty());
    MINI_CHECK(bare.empty());
    MINI_CHECK(session.get_interaction(a, b)[0]->guid() == glue->guid());
    MINI_CHECK(session.get_interaction(b, a)[0]->guid() == glue->guid());
    MINI_CHECK(session.get_interaction(a, b)[0]->name == "glue");
    MINI_CHECK(duplicate.get_interaction(b, a)[0]->name == "screw");
    MINI_CHECK(duplicate.get_interaction(b, a)[0]->guid() == glue->guid());
    MINI_CHECK(*loaded_b.get_interaction(b, a)[0] == *glue);
    MINI_CHECK(loaded_b.get_interaction(b, a)[0]->guid() == glue->guid());
    MINI_CHECK(*loaded_j.get_interaction(b, a)[0] == *glue);
}

MINI_TEST("Session", "Has Interaction") {

    Interaction::register_type("NamedInteraction", named_interaction);
    Session session;
    const std::shared_ptr<Element> a = std::make_shared<Element>("a");
    const std::shared_ptr<Element> b = std::make_shared<Element>("b");
    const std::shared_ptr<Element> absent = std::make_shared<Element>("absent");
    session.add_element(a);
    session.add_element(b);
    const bool before = session.has_interaction(a, b);
    session.add_interaction(a, b, std::make_shared<NamedInteraction>());
    const Session loaded = Session::pb_loads(session.pb_dumps());

    MINI_CHECK(!before);
    MINI_CHECK(session.has_interaction(a, b));
    MINI_CHECK(session.has_interaction(b, a));
    MINI_CHECK(!session.has_interaction(a, absent));
    MINI_CHECK(loaded.has_interaction(b, a));
}

MINI_TEST("Session", "Remove Interaction") {

    Session session;
    const std::shared_ptr<Element> a = std::make_shared<Element>("a");
    const std::shared_ptr<Element> b = std::make_shared<Element>("b");
    const std::shared_ptr<Element> c = std::make_shared<Element>("c");
    session.add_element(a);
    session.add_element(b);
    session.add_element(c);
    session.add_interaction(a, b, std::make_shared<NamedInteraction>("glue"));
    session.add_interaction(a, c, std::make_shared<NamedInteraction>());
    session.remove_interaction(b, a);
    session.remove_interaction(b, a);

    MINI_CHECK(!session.has_interaction(a, b));
    MINI_CHECK(session.has_interaction(a, c));
    MINI_CHECK(session.get_interaction(a, b).empty());
    MINI_CHECK(session.interactions.size() == 1);
    MINI_CHECK(session.graph.number_of_edges() == 1);
    MINI_CHECK(session.graph.has_node(b->guid()));
}

MINI_TEST("Session", "Undo Remove Interaction") {

    Session session;
    const std::shared_ptr<Element> a = std::make_shared<Element>("a");
    const std::shared_ptr<Element> b = std::make_shared<Element>("b");
    session.add_element(a);
    session.add_element(b);
    const std::shared_ptr<Interaction> glue = session.add_interaction(a, b, std::make_shared<NamedInteraction>("glue"));
    const std::string id = session.graph.edges.at(a->guid()).at(b->guid()).guid();

    session.begin("remove");
    session.remove_object(b->guid());
    session.commit();
    const bool dropped = session.interactions.count(id) == 0;
    session.undo();

    MINI_CHECK(dropped);
    MINI_CHECK(session.graph.edges[a->guid()][b->guid()].guid() == id);
    MINI_CHECK(session.get_interaction(a, b).size() == 1);
    MINI_CHECK(session.get_interaction(a, b)[0]->guid() == glue->guid());
    MINI_CHECK(session.get_interaction(a, b)[0]->name == "glue");

    session.redo();

    MINI_CHECK(session.interactions.count(id) == 0);
}

MINI_TEST("Session", "Get Neighbours") {

    Session session;
    std::shared_ptr<Point> p1 = std::make_shared<Point>(0, 0, 0);
    std::shared_ptr<Point> p2 = std::make_shared<Point>(1, 0, 0);
    session.add_point(p1);
    session.add_point(p2);
    session.add_edge(p1->guid(), p2->guid(), "connection");

    std::vector<std::string> neighbours = session.get_neighbours(p1->guid());

    MINI_CHECK(neighbours.size() == 1);
    MINI_CHECK(neighbours[0] == p2->guid());
}

MINI_TEST("Session", "Get Collisions") {

    Session session;
    std::shared_ptr<OBB> obb1 = std::make_shared<OBB>(
        Point(0.0, 0.0, 0.0),
        Vector(1.0, 0.0, 0.0),
        Vector(0.0, 1.0, 0.0),
        Vector(0.0, 0.0, 1.0),
        Vector(2.0, 2.0, 2.0)
    );
    std::shared_ptr<OBB> obb2 = std::make_shared<OBB>(
        Point(1.0, 0.0, 0.0),
        Vector(1.0, 0.0, 0.0),
        Vector(0.0, 1.0, 0.0),
        Vector(0.0, 0.0, 1.0),
        Vector(2.0, 2.0, 2.0)
    );
    session.add_obb(obb1);
    session.add_obb(obb2);
    std::vector<std::pair<std::string, std::string>> pairs = session.get_collisions();

    MINI_CHECK(pairs.size() >= 1);
}

MINI_TEST("Session", "Ray Cast") {

    Session session;
    std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
    mesh->add_vertex(Point(-1.0, -1.0, 0.0), 0);
    mesh->add_vertex(Point(1.0, -1.0, 0.0), 1);
    mesh->add_vertex(Point(0.0, 1.0, 0.0), 2);
    mesh->add_face(std::vector<size_t>{0, 1, 2});
    session.add_mesh(mesh);
    std::vector<Session::RayHit> hits = session.ray_cast(Point(0.0, 0.0, 2.0), Vector(0.0, 0.0, -1.0));

    MINI_CHECK(hits.size() >= 1);

    std::shared_ptr<Mesh> placed = std::make_shared<Mesh>();
    placed->add_vertex(Point(-1.0, -1.0, 0.0), 0);
    placed->add_vertex(Point(1.0, -1.0, 0.0), 1);
    placed->add_vertex(Point(0.0, 1.0, 0.0), 2);
    placed->add_face(std::vector<size_t>{0, 1, 2});
    std::string placed_guid = placed->guid();
    session.add_mesh(placed);
    session.set_xform(placed_guid, Xform::translation(100.0, 0.0, 0.0));
    std::vector<Session::RayHit> hits2 = session.ray_cast(Point(100.0, 0.0, 2.0), Vector(0.0, 0.0, -1.0));

    MINI_CHECK(hits2.size() >= 1);
    MINI_CHECK(TOLERANCE.is_close(hits2[0].hit_point[0], 100.0));
}

MINI_TEST("Session", "Get Object") {

    Session session;
    std::shared_ptr<Point> point = std::make_shared<Point>(1.0, 2.0, 3.0);
    session.add_point(point);

    std::shared_ptr<Point> retrieved = session.get_object<Point>(point->guid());

    MINI_CHECK(retrieved != nullptr);
    MINI_CHECK(retrieved->guid() == point->guid());
}

MINI_TEST("Session", "Remove Object") {

    Session session;
    std::shared_ptr<Point> point = std::make_shared<Point>(1.0, 2.0, 3.0);
    session.add_point(point);
    bool removed = session.remove_object(point->guid());

    std::shared_ptr<Element> plate = std::make_shared<Element>("p1");
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
    MINI_CHECK(session.objects.elements->number_of_slots() == 0);
    MINI_CHECK(session.number_of_dead() == 0);
    MINI_CHECK(!session.graph.has_node(eguid));
    MINI_CHECK(loaded.lookup.count(eguid) == 0);
    MINI_CHECK(loaded.objects.points->number_of_slots() == 0);
}

MINI_TEST("Session", "Get Geometry") {

    Session session;
    std::shared_ptr<Point> point = std::make_shared<Point>(1.0, 2.0, 3.0);
    session.add_point(point);

    Objects geom = session.get_geometry();

    MINI_CHECK(geom.points->size() == 1);
}

MINI_TEST("Session", "Get Geometry Is Pure") {

    Session session;
    std::shared_ptr<Point> point = std::make_shared<Point>(1.0, 2.0, 3.0);
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

    Session session;
    std::shared_ptr<Point> p1 = std::make_shared<Point>(1.0, 2.0, 3.0);
    std::shared_ptr<Point> p2 = std::make_shared<Point>(4.0, 5.0, 6.0);
    session.add_point(p1);
    session.add_point(p2);
    session.add_edge(p1->guid(), p2->guid(), "connection");
    session.set_xform(p1->guid(), Xform::translation(1.0, 0.0, 0.0));
    const std::string definition = session.add_definition(std::make_shared<Point>(0.0, 0.0, 0.0));
    std::shared_ptr<InstanceRef> instance = std::make_shared<InstanceRef>(definition, Xform::translation(0.0, 1.0, 0.0));
    session.add_instance(instance, Xform::translation(2.0, 0.0, 0.0));

    std::string fname = "serialization/test_session.json";
    session.file_json_dump(fname);
    Session loaded = Session::file_json_load(fname);

    MINI_CHECK(loaded.name == session.name);
    MINI_CHECK(loaded.lookup.size() == session.lookup.size());
    MINI_CHECK(loaded.graph.number_of_vertices() == session.graph.number_of_vertices());
    MINI_CHECK(loaded.xforms.at(p1->guid()).guid() == session.xforms.at(p1->guid()).guid());
    MINI_CHECK(loaded.xforms.at(instance->guid()).guid() == session.xforms.at(instance->guid()).guid());
    MINI_CHECK(loaded.objects.instances->at(0)->xform.guid() == instance->xform.guid());
}

MINI_TEST("Session", "Protobuf Roundtrip") {

    Session session;
    std::shared_ptr<Point> p1 = std::make_shared<Point>(1.0, 2.0, 3.0);
    std::shared_ptr<Point> p2 = std::make_shared<Point>(4.0, 5.0, 6.0);
    session.add_point(p1);
    session.add_point(p2);
    session.add_edge(p1->guid(), p2->guid(), "connection");

    std::string fname = "serialization/test_session.bin";
    session.pb_dump(fname);
    Session loaded = Session::pb_load(fname);
    Session converted = Session::from_proto(session.to_proto());

    MINI_CHECK(loaded.name == session.name);
    MINI_CHECK(loaded.lookup.size() == session.lookup.size());
    MINI_CHECK(converted.lookup.size() == session.lookup.size());
    MINI_CHECK(converted.graph.has_edge({p1->guid(), p2->guid()}));
}

MINI_TEST("Session", "Lookup Mutation Roundtrip") {

    Session session;
    std::shared_ptr<Line> line = std::make_shared<Line>(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
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

    Session session;
    std::shared_ptr<Line> line = std::make_shared<Line>(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    std::shared_ptr<Point> point = std::make_shared<Point>(1.0, 2.0, 3.0);
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

    Session session;
    std::shared_ptr<Point> point = std::make_shared<Point>(1.0, 2.0, 3.0);
    std::string guid = point->guid();
    session.add_point(point);

    Xform shift = Xform::translation(5.0, 0.0, 0.0);
    session.set_xform(guid, shift);

    MINI_CHECK(session.xform(guid) == shift);
    MINI_CHECK(session.world_xform(guid) == shift);
    MINI_CHECK(session.world_xforms()[guid] == shift);
    MINI_CHECK(session.xform("missing") == Xform::identity());
    MINI_CHECK(session.remove_xform(guid));
    MINI_CHECK(session.xform(guid) == Xform::identity());
}

MINI_TEST("Session", "World Xform Hierarchy") {

    Session session;
    std::shared_ptr<Point> a = std::make_shared<Point>(0.0, 0.0, 0.0);
    std::shared_ptr<Point> b = std::make_shared<Point>(0.0, 0.0, 0.0);
    std::shared_ptr<Point> c = std::make_shared<Point>(0.0, 0.0, 0.0);
    std::string a_guid = a->guid();
    std::string b_guid = b->guid();
    std::string c_guid = c->guid();
    std::shared_ptr<TreeNode> a_node = session.add_point(a);
    std::shared_ptr<TreeNode> b_node = session.add_point(b);
    std::shared_ptr<TreeNode> c_node = session.add_point(c);

    session.add(a_node);
    session.add(b_node, a_node);
    session.add(c_node, b_node);

    Xform a_xform = Xform::rotation_z(Tolerance::PI / 2.0);
    Xform b_xform = Xform::translation(2.0, 0.0, 0.0);
    Xform c_xform = Xform::rotation_z(Tolerance::PI / 2.0);
    session.set_xform(a_guid, a_xform);
    session.set_xform(b_guid, b_xform);
    session.set_xform(c_guid, c_xform);

    std::unordered_map<std::string, Xform> world = session.world_xforms();

    MINI_CHECK(session.world_xform(a_guid) == a_xform);
    MINI_CHECK(session.world_xform(b_guid) == a_xform * b_xform);
    MINI_CHECK(session.world_xform(c_guid) == a_xform * b_xform * c_xform);
    MINI_CHECK(world[c_guid] == session.world_xform(c_guid));
}

MINI_TEST("Session", "Xform Roundtrip") {

    Session session;
    std::shared_ptr<Point> point = std::make_shared<Point>(1.0, 2.0, 3.0);
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

/// A cube mesh of the given size centred on center.
static std::shared_ptr<Mesh> create_box(const Point& center, double size) {

    std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
    const double h = size * 0.5;
    const std::vector<Point> verts = {
        Point(center[0] - h, center[1] - h, center[2] - h),
        Point(center[0] + h, center[1] - h, center[2] - h),
        Point(center[0] + h, center[1] + h, center[2] - h),
        Point(center[0] - h, center[1] + h, center[2] - h),
        Point(center[0] - h, center[1] - h, center[2] + h),
        Point(center[0] + h, center[1] - h, center[2] + h),
        Point(center[0] + h, center[1] + h, center[2] + h),
        Point(center[0] - h, center[1] + h, center[2] + h)
    };

    for (size_t i = 0; i < verts.size(); ++i)
        mesh->add_vertex(verts[i], i);

    const std::vector<std::vector<size_t>> faces =
        {{0, 1, 2, 3}, {4, 7, 6, 5}, {0, 4, 5, 1}, {2, 6, 7, 3}, {0, 3, 7, 4}, {1, 5, 6, 2}};

    for (const std::vector<size_t>& f : faces)
        mesh->add_face(f);

    return mesh;
}

MINI_TEST("Session", "Tree Transformation Hierarchy") {

    Session scene("tree_transformation_test");

    std::shared_ptr<Mesh> box1 = create_box(Point(0, 0, 0), 2.0);
    std::string box1_guid = box1->guid();
    std::shared_ptr<TreeNode> box1_node = scene.add_mesh(box1);
    std::shared_ptr<Mesh> box2 = create_box(Point(0, 0, 0), 2.0);
    std::string box2_guid = box2->guid();
    std::shared_ptr<TreeNode> box2_node = scene.add_mesh(box2);
    std::shared_ptr<Mesh> box3 = create_box(Point(0, 0, 0), 2.0);
    std::string box3_guid = box3->guid();
    std::shared_ptr<TreeNode> box3_node = scene.add_mesh(box3);

    scene.add(box1_node);
    scene.add(box2_node, box1_node);
    scene.add(box3_node, box2_node);

    Plane plane_from(Point(0, 0, 0), Vector(1, 0, 0), Vector(0, 1, 0));
    Plane plane_to(Point(0, 0, 1.0), Vector(1, 0, 0), Vector(0, 1, 0));
    Xform xy_to_top = Xform::plane_to_plane(plane_from, plane_to);
    scene.set_xform(box1_guid, Xform::rotation_z(Tolerance::PI / 1.5) * xy_to_top);
    scene.set_xform(box2_guid, Xform::translation(2.0, 0, 0) * Xform::rotation_z(Tolerance::PI / 6.0));
    scene.set_xform(box3_guid, Xform::translation(2.0, 0, 0));

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

    Session session;

    Component c;
    c.type_name = "FloorBuilder";
    c.name = "floor_builder";
    c.extra = {{"size", 3000}, {"height", 650}};
    std::string guid = c.guid();

    session.add_component(c);

    MINI_CHECK(session.objects.components->size() == 1);
    MINI_CHECK(session.component_lookup.count(guid) == 1);
    MINI_CHECK(session.graph.has_node(guid));
}

MINI_TEST("Session", "Component Json Roundtrip") {

    Session original;
    Component c;
    c.type_name = "FloorBuilder";
    c.name = "floor_builder";
    c.extra = {{"size", 3000}, {"height", 650}, {"rise", 453}};
    std::string guid = c.guid();
    original.add_component(c);

    std::string filename = "serialization/test_session_component.json";
    file_encoders::file_json_dump(original, filename);
    Session loaded = file_encoders::file_json_load<Session>(filename);

    MINI_CHECK(loaded.objects.components->size() == 1);
    MINI_CHECK(loaded.objects.components->at(0).type_name == "FloorBuilder");
    MINI_CHECK(loaded.objects.components->at(0).extra["size"] == 3000);
    MINI_CHECK(loaded.objects.components->at(0).guid() == guid);
}

MINI_TEST("Session", "Document Workflow") {

    Session session;
    std::shared_ptr<Point> a = std::make_shared<Point>(1.0, 0.0, 0.0);
    std::shared_ptr<Point> b = std::make_shared<Point>(2.0, 0.0, 0.0);
    std::shared_ptr<Point> c = std::make_shared<Point>(3.0, 0.0, 0.0);
    std::string a_guid = a->guid();
    std::string b_guid = b->guid();
    std::string c_guid = c->guid();
    session.add_point(a);
    session.add_point(b);
    session.add_point(c);

    session.replace(b_guid, std::make_shared<Point>(20.0, 0.0, 0.0));
    session.remove_object(c_guid);
    Xform shift = Xform::translation(0.0, 5.0, 0.0);
    session.set_xform(a_guid, shift);

    std::string fname = "serialization/test_session_document.bin";
    session.pb_dump(fname);
    Session loaded = Session::pb_load(fname);

    MINI_CHECK(loaded.lookup.size() == 2);
    MINI_CHECK(loaded.lookup.count(a_guid) == 1);
    MINI_CHECK(loaded.lookup.count(b_guid) == 1);
    MINI_CHECK(loaded.lookup.count(c_guid) == 0);
    MINI_CHECK(TOLERANCE.is_close((*std::get<std::shared_ptr<Point>>(loaded.lookup[b_guid]))[0], 20.0));
    MINI_CHECK(loaded.xform(a_guid) == shift);
    MINI_CHECK(loaded.history.depth() == 0);
    MINI_CHECK(session.objects.points->size() == 2);
    MINI_CHECK(session.objects.points->number_of_slots() == 2);
}

MINI_TEST("Session", "Undo Remove") {

    Session session;
    std::shared_ptr<TreeNode> group = session.add_group("g");
    std::shared_ptr<Point> a = std::make_shared<Point>(1.0, 0.0, 0.0);
    std::shared_ptr<Point> b = std::make_shared<Point>(2.0, 0.0, 0.0);
    std::shared_ptr<Point> c = std::make_shared<Point>(3.0, 0.0, 0.0);
    std::string a_guid = a->guid();
    std::string b_guid = b->guid();
    std::string c_guid = c->guid();
    session.add_point(a, group);
    std::shared_ptr<TreeNode> b_node = session.add_point(b, group);
    session.add_point(c, b_node);
    session.add_edge(a_guid, b_guid, "connection");
    Xform shift = Xform::translation(0.0, 5.0, 0.0);
    session.set_xform(b_guid, shift);
    const std::shared_ptr<Point> stored = session.objects.points->at(1);

    session.begin("remove");
    session.remove_object(b_guid);
    session.commit();
    bool gone = session.lookup.count(b_guid) == 0 && group->children().size() == 1;
    session.undo();

    MINI_CHECK(gone);
    MINI_CHECK(session.lookup.count(b_guid) == 1);
    MINI_CHECK(session.objects.points->at(1)->guid() == b_guid);
    MINI_CHECK(session.objects.points->at(1) == stored);
    MINI_CHECK(group->children()[1]->name == b_guid);
    MINI_CHECK(group->children()[1] == b_node.get());
    MINI_CHECK(group->children()[1]->children()[0]->name == c_guid);
    MINI_CHECK(session.graph.has_edge({a_guid, b_guid}));
    MINI_CHECK(session.graph.edge_label(a_guid, b_guid) == "connection");
    MINI_CHECK(session.xform(b_guid) == shift);

    session.redo();

    MINI_CHECK(session.lookup.count(b_guid) == 0);
    MINI_CHECK(session.objects.points->size() == 2);
    MINI_CHECK(group->children().size() == 1);
    MINI_CHECK(!session.graph.has_edge({a_guid, b_guid}));
    MINI_CHECK(session.xform(b_guid) == Xform::identity());
}

MINI_TEST("Session", "Undo Add") {

    Session session;
    std::shared_ptr<TreeNode> group = session.add_group("g");
    session.add_point(std::make_shared<Point>(0.0, 0.0, 0.0), group);
    std::shared_ptr<Point> point = std::make_shared<Point>(1.0, 2.0, 3.0);
    std::string guid = point->guid();

    session.begin("add");
    session.add_point(point, group);
    session.commit();
    session.undo();
    bool gone = session.lookup.count(guid) == 0 && session.objects.points->size() == 1;
    session.redo();

    MINI_CHECK(gone);
    MINI_CHECK(session.lookup.count(guid) == 1);
    MINI_CHECK(session.objects.points->at(1)->guid() == guid);
    MINI_CHECK(session.objects.points->number_of_slots() == 2);
    MINI_CHECK(group->children()[1]->name == guid);
    MINI_CHECK(session.graph.has_node(guid));
    MINI_CHECK(TOLERANCE.is_close((*std::get<std::shared_ptr<Point>>(session.lookup[guid]))[2], 3.0));
}

MINI_TEST("Session", "Undo Replace") {

    Session session;
    std::shared_ptr<Point> point = std::make_shared<Point>(1.0, 2.0, 3.0);
    std::string guid = point->guid();
    session.add_point(point);

    session.begin("replace");
    session.replace(guid, std::make_shared<Point>(9.0, 9.0, 9.0));
    session.commit();
    double replaced = (*std::get<std::shared_ptr<Point>>(session.lookup[guid]))[0];
    session.undo();
    double restored = (*std::get<std::shared_ptr<Point>>(session.lookup[guid]))[0];
    session.redo();

    MINI_CHECK(TOLERANCE.is_close(replaced, 9.0));
    MINI_CHECK(TOLERANCE.is_close(restored, 1.0));
    MINI_CHECK(TOLERANCE.is_close((*std::get<std::shared_ptr<Point>>(session.lookup[guid]))[0], 9.0));
    MINI_CHECK(session.objects.points->at(0)->guid() == guid);
    MINI_CHECK(session.objects.points->size() == 1);
    MINI_CHECK(session.objects.points->number_of_slots() == 1);
}

MINI_TEST("Session", "Undo Xform") {

    Session session;
    std::shared_ptr<Point> point = std::make_shared<Point>(1.0, 2.0, 3.0);
    std::string guid = point->guid();
    session.add_point(point);
    Xform shift = Xform::translation(5.0, 0.0, 0.0);

    session.begin("move");
    session.set_xform(guid, shift);
    session.commit();
    session.undo();
    bool cleared = session.xform(guid) == Xform::identity();
    session.redo();

    session.begin("reset");
    session.remove_xform(guid);
    session.commit();
    session.undo();

    MINI_CHECK(cleared);
    MINI_CHECK(session.xform(guid) == shift);
    MINI_CHECK(session.xforms.size() == 1);
    MINI_CHECK(std::get<XformOp>(session.history.undo_stack[0].ops[0]).kind == "xform");
}

MINI_TEST("Session", "History Purged On Save") {

    Session session;

    session.begin("add");
    session.add_point(std::make_shared<Point>(0.0, 0.0, 0.0));
    session.commit();
    int before_pb = session.history.depth();
    session.pb_dumps();
    int after_pb = session.history.depth();

    session.begin("add");
    session.add_point(std::make_shared<Point>(1.0, 0.0, 0.0));
    session.commit();
    int before_json = session.history.depth();
    session.file_json_dumps();

    MINI_CHECK(before_pb == 1);
    MINI_CHECK(after_pb == 0);
    MINI_CHECK(before_json == 1);
    MINI_CHECK(session.history.depth() == 0);
    MINI_CHECK(session.number_of_dead() == 0);
    MINI_CHECK(!session.undo());
    MINI_CHECK(session.objects.points->size() == 2);
}

MINI_TEST("Session", "Purge On Save") {

    Session session;
    std::vector<std::string> guids;

    for (int i = 0; i < 5; ++i)
        guids.push_back(session.add_point(std::make_shared<Point>(static_cast<double>(i), 0.0, 0.0))->name);

    for (int i : {1, 3}) {
        session.begin("remove");
        session.remove_object(guids[i]);
        session.commit();
    }

    const std::string bytes = session.pb_dumps();
    Session loaded = Session::pb_loads(bytes);
    std::vector<int> indices;

    for (const Vertex& vertex : session.graph.get_vertices())
        indices.push_back(vertex.index);

    std::sort(indices.begin(), indices.end());

    MINI_CHECK(session.history.depth() == 0);
    MINI_CHECK(session.number_of_dead() == 0);
    MINI_CHECK(session.objects.points->number_of_slots() == 3);
    MINI_CHECK(indices == std::vector<int>({0, 1, 2}));
    MINI_CHECK(loaded.order() == std::vector<std::string>({guids[0], guids[2], guids[4]}));
    MINI_CHECK(loaded.pb_dumps() == bytes);
}

MINI_TEST("Session", "Purge Unreachable") {

    Session session;
    std::vector<std::string> guids;

    for (int i = 0; i < 70; ++i)
        guids.push_back(session.add_point(std::make_shared<Point>(static_cast<double>(i), 0.0, 0.0))->name);

    for (const std::string& guid : guids) {
        session.begin("remove");
        session.remove_object(guid);
        session.commit();
    }

    const bool due = session.purge_due();

    while (session.purge_step(PURGE_WORK)) {}

    const size_t dead = session.number_of_dead();
    int undone = 0;

    while (session.undo())
        undone++;

    MINI_CHECK(due);
    MINI_CHECK(dead == 64);
    MINI_CHECK(undone == 64);
    MINI_CHECK(session.objects.points->size() == 64);
}

MINI_TEST("Session", "Purge Step") {

    Session session;
    std::vector<std::string> guids;

    for (int i = 0; i < 10000; ++i)
        guids.push_back(session.add_point(std::make_shared<Point>(static_cast<double>(i), 0.0, 0.0))->name);

    session.begin("remove");

    for (size_t i = 0; i < guids.size(); i += 2)
        session.remove_object(guids[i]);

    session.commit();

    for (int i = 0; i < 64; ++i) {
        session.begin("move");
        session.set_xform(guids[1], Xform::translation(static_cast<double>(i), 0.0, 0.0));
        session.commit();
    }

    std::vector<std::string> odd;

    for (size_t i = 1; i < guids.size(); i += 2)
        odd.push_back(guids[i]);

    std::vector<std::string> expected = odd;
    const bool first = session.purge_step(64);
    bool ordered = session.order() == expected;
    int calls = 1;

    while (session.purge_step(64)) {

        calls++;

        if (calls == 10) {
            session.begin("remove");
            session.remove_object(guids[3]);
            session.commit();
            expected.erase(std::find(expected.begin(), expected.end(), guids[3]));
        }

        if (calls == 20) {
            session.undo();
            expected = odd;
        }

        if (calls == 30)
            expected.push_back(session.add_point(std::make_shared<Point>(0.0, 1.0, 0.0))->name);

        ordered &= session.order() == expected;
    }

    MINI_CHECK(first);
    MINI_CHECK(calls > 30);
    MINI_CHECK(ordered);
    MINI_CHECK(session.order() == expected);
    MINI_CHECK(session.number_of_dead() == 0);
    MINI_CHECK(session.objects.points->number_of_slots() == session.objects.points->size());
    MINI_CHECK(session.tree.root()->children().size() == expected.size());
}

MINI_TEST("Session", "Checkpoint Keeps History") {

    Session session;
    const std::shared_ptr<TreeNode> group = session.add_group("group");
    const std::shared_ptr<Point> a = std::make_shared<Point>(0.0, 0.0, 0.0);
    const std::shared_ptr<Point> b = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::shared_ptr<Point> c = std::make_shared<Point>(2.0, 0.0, 0.0);
    const std::string a_guid = a->guid();
    const std::string b_guid = b->guid();
    const std::string c_guid = c->guid();
    session.add_point(a, group);
    session.add_point(b, group);
    session.add_point(c);
    session.set_node_color(group, Color(1.0, 0.0, 0.0, 1.0));
    session.set_xform("group", Xform::translation(0.0, 0.0, 1.0));
    session.set_xform(c_guid, Xform::translation(5.0, 0.0, 0.0));
    session.add_edge(a_guid, c_guid, "touch");
    const std::string whole = session.checkpoint(SIZE_MAX).value();
    const std::string copy = Session(session).pb_dumps();

    session.begin("remove");
    session.remove_object(b_guid);
    session.commit();
    std::optional<std::string> bytes;
    int calls = 0;

    while (!bytes) {
        bytes = session.checkpoint(16);
        calls++;
    }

    const Session loaded = Session::pb_loads(*bytes);
    session_proto::Session parsed;
    MINI_CHECK(parsed.ParseFromString(*bytes));

    MINI_CHECK(whole == copy);
    MINI_CHECK(calls > 1);
    MINI_CHECK(google::protobuf::util::MessageDifferencer::Equals(parsed, session.to_proto()));
    MINI_CHECK(session.history.can_undo());
    MINI_CHECK(loaded.lookup.count(b_guid) == 0);
    MINI_CHECK(loaded.order() == session.order());
    MINI_CHECK(session.undo());
    MINI_CHECK(session.lookup.count(b_guid) == 1);
}

MINI_TEST("Session", "Checkpoint Restarts On Edit") {

    Session session;
    std::vector<std::string> guids;

    for (int i = 0; i < 20; ++i)
        guids.push_back(session.add_point(std::make_shared<Point>(static_cast<double>(i), 0.0, 0.0))->name);

    for (int i = 0; i < 25; ++i) {
        const std::string name = session.add_group("group" + std::to_string(i))->name;
        session.set_xform(name, Xform::translation(0.0, static_cast<double>(i + 1), 0.0));
    }

    const std::optional<std::string> first = session.checkpoint(10);
    session.begin("remove");
    session.remove_object(guids[5]);
    session.commit();
    std::optional<std::string> bytes;

    while (!bytes)
        bytes = session.checkpoint(10);

    const Session loaded = Session::pb_loads(*bytes);
    session_proto::Session parsed;
    MINI_CHECK(parsed.ParseFromString(*bytes));

    MINI_CHECK(!first);
    MINI_CHECK(google::protobuf::util::MessageDifferencer::Equals(parsed, session.to_proto()));
    MINI_CHECK(loaded.objects.points->size() == 19);
    MINI_CHECK(loaded.xforms.size() == 25);
    MINI_CHECK(loaded.lookup.count(guids[5]) == 0);
    MINI_CHECK(session.history.can_undo());
}

MINI_TEST("Session", "History Capacity") {

    Session session;

    for (int i = 0; i < 70; ++i) {
        session.begin("add");
        session.add_point(std::make_shared<Point>(double(i), 0.0, 0.0));
        session.commit();
    }

    int depth = session.history.depth();

    while (session.undo()) {
    }

    MINI_CHECK(depth == 64);
    MINI_CHECK(!session.history.can_undo());
    MINI_CHECK(session.objects.points->size() == 6);
    MINI_CHECK(session.objects.points->number_of_slots() == 70);
    MINI_CHECK(session.history.dropped == 6);
    MINI_CHECK(TOLERANCE.is_close((*session.objects.points->at(5))[0], 5.0));
}

MINI_TEST("Session", "Str Hierarchy") {

    Session session("blocks");
    std::shared_ptr<TreeNode> group = session.add_group("Group");
    session.add_point(std::make_shared<Point>(0.0, 0.0, 0.0), group);
    session.add_point(std::make_shared<Point>(1.0, 0.0, 0.0), group);
    const std::string text = session.str();

    MINI_CHECK(text.find("Spatial Hierarchy") != std::string::npos);
    MINI_CHECK(text.find("Element Interactions") != std::string::npos);
    MINI_CHECK(text.find("\u2514\u2500\u2500 ") != std::string::npos);
    MINI_CHECK(text.find("<Tree with ") != std::string::npos);
    MINI_CHECK(text.find("<Graph with ") != std::string::npos);
    MINI_CHECK(session.repr().rfind("Session(name=blocks", 0) == 0);
}

MINI_TEST("Session", "Add Definition") {

    Session session;
    std::shared_ptr<Mesh> box = create_box(Point(0, 0, 0), 2.0);
    std::string guid = session.add_definition(box);
    std::string again = session.add_definition(box);
    session.set_xform(guid, Xform::translation(1.0, 0.0, 0.0));
    std::shared_ptr<Point> point = std::make_shared<Point>(1.0, 2.0, 3.0);
    session.add_point(point);
    std::string taken = session.add_definition(point);

    MINI_CHECK(guid == box->guid());
    MINI_CHECK(again == guid);
    MINI_CHECK(taken.empty());
    MINI_CHECK(session.definitions.meshes->size() == 1);
    MINI_CHECK(session.definition_lookup.count(guid) == 1);
    MINI_CHECK(session.lookup.count(guid) == 0);
    MINI_CHECK(session.order().size() == 1);
    MINI_CHECK(!session.graph.has_node(guid));
    MINI_CHECK(session.tree.get_node_by_name(guid) == nullptr);
    MINI_CHECK(session.xforms.empty());
}

MINI_TEST("Session", "Add Instance") {

    Session session;
    std::shared_ptr<TreeNode> group = session.add_group("bay");
    std::string definition = session.add_definition(create_box(Point(0, 0, 0), 2.0));
    std::shared_ptr<InstanceRef> instance = std::make_shared<InstanceRef>(definition, Xform::translation(0.0, 0.0, 3.0));
    instance->name = "column";
    std::string guid = instance->guid();
    std::shared_ptr<TreeNode> node = session.add_instance(instance, Xform::translation(10.0, 0.0, 0.0), group);
    std::shared_ptr<TreeNode> orphan = session.add_instance(std::make_shared<InstanceRef>("missing", Xform::identity()));

    MINI_CHECK(node->name == guid);
    MINI_CHECK(group->children()[0]->name == guid);
    MINI_CHECK(session.graph.node_label(guid) == "instance_column");
    MINI_CHECK(session.objects.instances->size() == 1);
    MINI_CHECK(session.instance_lookup[guid]->xform == Xform::identity());
    MINI_CHECK(session.xform(guid) == Xform::translation(10.0, 0.0, 3.0));
    MINI_CHECK(orphan == nullptr);
    MINI_CHECK(session.order().empty());
}

MINI_TEST("Session", "Definition Of") {

    Session session;
    std::string definition = session.add_definition(create_box(Point(0, 0, 0), 2.0));
    std::shared_ptr<InstanceRef> instance = std::make_shared<InstanceRef>(definition, Xform::identity());
    std::string guid = instance->guid();
    session.add_instance(instance);
    std::optional<Geometry> found = session.definition_of(guid);

    MINI_CHECK(found.has_value());
    MINI_CHECK(std::get<std::shared_ptr<Mesh>>(*found)->guid() == definition);
    MINI_CHECK(!session.definition_of(definition).has_value());
    MINI_CHECK(!session.definition_of("missing").has_value());
}

MINI_TEST("Session", "Instances Of") {

    Session session;
    std::string definition = session.add_definition(create_box(Point(0, 0, 0), 2.0));
    std::shared_ptr<InstanceRef> first = std::make_shared<InstanceRef>(definition, Xform::identity());
    std::shared_ptr<InstanceRef> second = std::make_shared<InstanceRef>(definition, Xform::identity());
    std::string first_guid = first->guid();
    std::string second_guid = second->guid();
    session.add_instance(first);
    session.add_instance(second);
    std::vector<std::string> guids = session.instances_of(definition);

    MINI_CHECK(guids.size() == 2);
    MINI_CHECK(guids[0] == first_guid);
    MINI_CHECK(guids[1] == second_guid);
    MINI_CHECK(session.instances_of("missing").empty());
}

MINI_TEST("Session", "World Geometry") {

    Session session;
    std::string definition = session.add_definition(create_box(Point(0, 0, 0), 2.0));
    std::shared_ptr<InstanceRef> instance = std::make_shared<InstanceRef>(definition, Xform::identity());
    instance->name = "box";
    std::string guid = instance->guid();
    session.add_instance(instance, Xform::translation(10.0, 0.0, 0.0));
    std::shared_ptr<Point> point = std::make_shared<Point>(1.0, 2.0, 3.0);
    session.add_point(point);
    session.set_xform(point->guid(), Xform::translation(0.0, 0.0, 5.0));

    std::shared_ptr<Mesh> mesh = std::get<std::shared_ptr<Mesh>>(*session.world_geometry(guid));
    std::shared_ptr<Point> moved = std::get<std::shared_ptr<Point>>(*session.world_geometry(point->guid()));
    std::shared_ptr<Mesh> local = std::get<std::shared_ptr<Mesh>>(session.definition_lookup[definition]);

    MINI_CHECK(mesh->guid() == guid);
    MINI_CHECK(mesh->name == "box");
    MINI_CHECK(TOLERANCE.is_close((*mesh->vertex_point(0))[0], 9.0));
    MINI_CHECK(TOLERANCE.is_close((*local->vertex_point(0))[0], -1.0));
    MINI_CHECK(TOLERANCE.is_close((*moved)[2], 8.0));
    MINI_CHECK(TOLERANCE.is_close((*point)[2], 3.0));
    MINI_CHECK(!session.world_geometry("missing").has_value());
}

MINI_TEST("Session", "Get Geometry Resolves Instances") {

    Session session;
    std::string definition = session.add_definition(create_box(Point(0, 0, 0), 2.0));
    std::shared_ptr<TreeNode> group = session.add_group("row");
    session.set_xform("row", Xform::translation(0.0, 5.0, 0.0));
    session.add_instance(std::make_shared<InstanceRef>(definition, Xform::identity()), Xform::translation(10.0, 0.0, 0.0), group);
    session.add_instance(std::make_shared<InstanceRef>(definition, Xform::identity()), Xform::translation(20.0, 0.0, 0.0), group);

    Objects geometry = session.get_geometry();
    Point corner = *geometry.meshes->at(1)->vertex_point(0);

    MINI_CHECK(geometry.instances->empty());
    MINI_CHECK(geometry.meshes->size() == 2);
    MINI_CHECK(TOLERANCE.is_close(corner[0], 19.0));
    MINI_CHECK(TOLERANCE.is_close(corner[1], 4.0));
    MINI_CHECK(session.objects.instances->size() == 2);
    MINI_CHECK(session.objects.meshes->empty());
}

MINI_TEST("Session", "Replace Definition") {

    Session session;
    std::string definition = session.add_definition(create_box(Point(0, 0, 0), 2.0));
    std::shared_ptr<InstanceRef> first = std::make_shared<InstanceRef>(definition, Xform::identity());
    std::shared_ptr<InstanceRef> second = std::make_shared<InstanceRef>(definition, Xform::identity());
    std::string second_guid = second->guid();
    session.add_instance(first, Xform::translation(10.0, 0.0, 0.0));
    session.add_instance(second, Xform::translation(20.0, 0.0, 0.0));

    bool replaced = session.replace_definition(definition, create_box(Point(0, 0, 0), 4.0));
    bool missing = session.replace_definition("missing", create_box(Point(0, 0, 0), 4.0));
    std::shared_ptr<Mesh> mesh = std::get<std::shared_ptr<Mesh>>(*session.world_geometry(second_guid));

    MINI_CHECK(replaced);
    MINI_CHECK(!missing);
    MINI_CHECK(session.definitions.meshes->size() == 1);
    MINI_CHECK(session.definitions.meshes->at(0)->guid() == definition);
    MINI_CHECK(TOLERANCE.is_close((*mesh->vertex_point(0))[0], 18.0));
}

MINI_TEST("Session", "Remove Definition") {

    Session session;
    std::string definition = session.add_definition(create_box(Point(0, 0, 0), 2.0));
    std::shared_ptr<InstanceRef> instance = std::make_shared<InstanceRef>(definition, Xform::identity());
    std::string guid = instance->guid();
    session.add_instance(instance);

    bool refused = !session.remove_definition(definition);
    session.remove_object(guid);
    bool removed = session.remove_definition(definition);

    MINI_CHECK(refused);
    MINI_CHECK(removed);
    MINI_CHECK(session.definitions.meshes->empty());
    MINI_CHECK(session.definitions.meshes->number_of_dead() == 1);
    MINI_CHECK(session.definition_lookup.empty());
    MINI_CHECK(session.objects.instances->empty());
    MINI_CHECK(!session.remove_definition("missing"));
}

MINI_TEST("Session", "To Instance") {

    Session session;
    std::shared_ptr<TreeNode> group = session.add_group("bay");
    std::string definition = session.add_definition(create_box(Point(0, 0, 0), 2.0));
    std::shared_ptr<Point> point = std::make_shared<Point>(0.0, 0.0, 0.0);
    std::shared_ptr<Mesh> box = create_box(Point(5.0, 0.0, 0.0), 2.0);
    box->name = "column";
    std::string guid = box->guid();
    session.add_point(point, group);
    session.add_mesh(box, group);
    session.add_edge(point->guid(), guid, "contact");
    session.set_xform(guid, Xform::translation(0.0, 0.0, 1.0));

    Point before = *std::get<std::shared_ptr<Mesh>>(*session.world_geometry(guid))->vertex_point(0);
    session.begin("to instance");
    bool converted = session.to_instance(guid, definition, Xform::translation(5.0, 0.0, 0.0));
    session.commit();
    Point after = *std::get<std::shared_ptr<Mesh>>(*session.world_geometry(guid))->vertex_point(0);

    MINI_CHECK(converted);
    MINI_CHECK(session.objects.meshes->empty());
    MINI_CHECK(session.objects.meshes->number_of_slots() == 1);
    MINI_CHECK(session.history.can_undo());
    MINI_CHECK(session.instance_lookup[guid]->name == "column");
    MINI_CHECK(session.instance_lookup[guid]->definition_guid == definition);
    MINI_CHECK(group->children()[1]->name == guid);
    MINI_CHECK(session.graph.has_edge({point->guid(), guid}));
    MINI_CHECK(session.graph.node_label(guid) == "instance_column");
    MINI_CHECK(TOLERANCE.is_close(before[0], after[0]));
    MINI_CHECK(TOLERANCE.is_close(before[2], after[2]));
    MINI_CHECK(!session.to_instance(guid, definition, Xform::identity()));
}

MINI_TEST("Session", "Explode") {

    Session session;
    std::string definition = session.add_definition(std::make_shared<Element>(*create_box(Point(0, 0, 0), 2.0), "plate"));
    std::shared_ptr<InstanceRef> instance = std::make_shared<InstanceRef>(definition, Xform::identity());
    instance->name = "deck";
    instance->features.push_back(ElementFeature("contact", 0, {Polyline(std::vector<Point>{Point(0, 0, 0), Point(1, 0, 0)})}));
    std::string guid = instance->guid();
    std::string feature = instance->features[0].guid();
    std::shared_ptr<Point> point = std::make_shared<Point>(0.0, 0.0, 0.0);
    session.add_point(point);
    session.add_instance(instance, Xform::translation(10.0, 0.0, 0.0));
    session.add_edge(point->guid(), guid, "contact");

    bool exploded = session.explode(guid);
    std::shared_ptr<Element> element = session.get_object<Element>(guid);

    MINI_CHECK(exploded);
    MINI_CHECK(session.objects.instances->empty());
    MINI_CHECK(session.objects.instances->number_of_dead() == 1);
    MINI_CHECK(element->name == "deck");
    MINI_CHECK(element->features().size() == 1);
    MINI_CHECK(element->features()[0].guid() == feature);
    MINI_CHECK(session.xform(guid) == Xform::translation(10.0, 0.0, 0.0));
    MINI_CHECK(session.graph.has_edge({point->guid(), guid}));
    MINI_CHECK(session.graph.node_label(guid) == "element_deck");
    MINI_CHECK(session.definitions.elements->size() == 1);
    MINI_CHECK(!session.explode(guid));
}

MINI_TEST("Session", "Undo Instance") {

    Session session;
    std::shared_ptr<TreeNode> group = session.add_group("bay");
    std::string definition = session.add_definition(create_box(Point(0, 0, 0), 2.0));
    std::shared_ptr<Point> point = std::make_shared<Point>(0.0, 0.0, 0.0);
    std::shared_ptr<Mesh> box = create_box(Point(0, 0, 0), 2.0);
    std::string guid = box->guid();
    session.add_point(point, group);
    session.add_mesh(box, group);
    session.add_edge(point->guid(), guid, "contact");
    std::string edge = session.graph.edges[point->guid()][guid].guid();
    std::vector<std::string> order = session.order();
    std::string tree = session.tree.str();
    std::string label = session.graph.node_label(guid);

    session.begin("to instance");
    session.to_instance(guid, definition, Xform::identity());
    session.commit();
    session.begin("explode");
    session.explode(guid);
    session.commit();
    session.undo();
    bool instanced = session.instance_lookup.count(guid) == 1 && session.tree.str() == tree;
    session.undo();

    std::shared_ptr<InstanceRef> instance = std::make_shared<InstanceRef>(definition, Xform::identity());
    std::string added = instance->guid();
    session.begin("add");
    session.add_instance(instance, Xform::translation(1.0, 0.0, 0.0), group);
    session.commit();
    session.undo();
    bool gone = session.instance_lookup.empty() && session.xforms.empty();
    session.redo();

    MINI_CHECK(instanced);
    MINI_CHECK(gone);
    MINI_CHECK(session.objects.meshes->at(0)->guid() == guid);
    MINI_CHECK(session.objects.meshes->number_of_slots() == 2);
    MINI_CHECK(session.objects.instances->size() == 1);
    MINI_CHECK(session.graph.has_edge({point->guid(), guid}));
    MINI_CHECK(session.graph.edges[guid][point->guid()].guid() == edge);
    MINI_CHECK(session.graph.node_label(guid) == label);
    MINI_CHECK(session.order() == order);
    MINI_CHECK(session.xform(added) == Xform::translation(1.0, 0.0, 0.0));
    MINI_CHECK(group->children()[2]->name == added);
}

MINI_TEST("Session", "Instance Json Roundtrip") {

    Session session;
    std::string definition = session.add_definition(create_box(Point(0, 0, 0), 2.0));
    std::shared_ptr<InstanceRef> instance = std::make_shared<InstanceRef>(definition, Xform::identity());
    std::string guid = instance->guid();
    session.add_instance(instance, Xform::translation(10.0, 0.0, 0.0));

    std::string fname = "serialization/test_session_instance.json";
    session.file_json_dump(fname);
    Session loaded = Session::file_json_load(fname);
    nlohmann::ordered_json data = session.jsondump();
    data["objects"]["instances"][0]["xform"] = Xform::translation(0.0, 0.0, 1.0).jsondump();
    Session folded = Session::jsonload(data);

    MINI_CHECK(loaded.definitions.meshes->size() == 1);
    MINI_CHECK(loaded.instance_lookup.count(guid) == 1);
    MINI_CHECK(loaded.definition_of(guid).has_value());
    MINI_CHECK(loaded.xform(guid) == Xform::translation(10.0, 0.0, 0.0));
    MINI_CHECK(folded.xform(guid) == Xform::translation(10.0, 0.0, 1.0));
    MINI_CHECK(folded.instance_lookup[guid]->xform == Xform::identity());
}

MINI_TEST("Session", "Instance Protobuf Roundtrip") {

    Session session;
    std::string definition = session.add_definition(create_box(Point(0, 0, 0), 2.0));
    std::shared_ptr<InstanceRef> instance = std::make_shared<InstanceRef>(definition, Xform::identity());
    instance->features.push_back(ElementFeature("contact", 0, {Polyline(std::vector<Point>{Point(0, 0, 0), Point(1, 0, 0)})}));
    std::string guid = instance->guid();
    std::string feature = instance->features[0].guid();
    session.add_instance(instance, Xform::translation(10.0, 0.0, 0.0));

    std::string fname = "serialization/test_session_instance.bin";
    session.pb_dump(fname);
    Session loaded = Session::pb_load(fname);
    session_proto::Session plain;
    MINI_CHECK(plain.ParseFromString(Session().pb_dumps()));

    MINI_CHECK(loaded.definitions.meshes->size() == 1);
    MINI_CHECK(loaded.instance_lookup[guid]->features.size() == 1);
    MINI_CHECK(loaded.instance_lookup[guid]->features[0].guid() == feature);
    MINI_CHECK(loaded.definition_of(guid).has_value());
    MINI_CHECK(loaded.xform(guid) == Xform::translation(10.0, 0.0, 0.0));
    MINI_CHECK(!plain.has_definitions());
}

MINI_TEST("Session", "Get Collisions Instances") {

    Session session;
    std::string definition = session.add_definition(create_box(Point(0, 0, 0), 2.0));
    std::shared_ptr<InstanceRef> first = std::make_shared<InstanceRef>(definition, Xform::identity());
    std::shared_ptr<InstanceRef> second = std::make_shared<InstanceRef>(definition, Xform::identity());
    std::shared_ptr<InstanceRef> third = std::make_shared<InstanceRef>(definition, Xform::identity());
    std::string first_guid = first->guid();
    std::string second_guid = second->guid();
    std::string third_guid = third->guid();
    session.add_instance(first);
    session.add_instance(second, Xform::translation(1.0, 0.0, 0.0));
    session.add_instance(third, Xform::translation(100.0, 0.0, 0.0));

    std::vector<std::pair<std::string, std::string>> pairs = session.get_collisions();

    MINI_CHECK(pairs.size() == 1);
    MINI_CHECK(session.graph.has_edge({first_guid, second_guid}));
    MINI_CHECK(!session.graph.has_edge({first_guid, third_guid}));
}

MINI_TEST("Session", "Ray Cast Instance") {

    Session session;
    std::string definition = session.add_definition(create_box(Point(0, 0, 0), 2.0));
    std::shared_ptr<InstanceRef> instance = std::make_shared<InstanceRef>(definition, Xform::identity());
    std::string guid = instance->guid();
    session.add_instance(instance, Xform::translation(100.0, 0.0, 0.0));

    std::vector<Session::RayHit> hits = session.ray_cast(Point(100.0, 0.0, 5.0), Vector(0.0, 0.0, -1.0));

    MINI_CHECK(hits.size() == 1);
    MINI_CHECK(hits[0].guid == guid);
    MINI_CHECK(TOLERANCE.is_close(hits[0].hit_point[0], 100.0));
    MINI_CHECK(TOLERANCE.is_close(hits[0].hit_point[2], 1.0));
}

MINI_TEST("Session", "Get Node") {

    Session session;
    const std::shared_ptr<TreeNode> node = session.add_point(std::make_shared<Point>(0.0, 0.0, 0.0));
    const std::string guid = node->name;
    const std::shared_ptr<TreeNode> child = session.add_point(std::make_shared<Point>(1.0, 0.0, 0.0), node);
    const std::string child_guid = child->name;
    const std::shared_ptr<TreeNode> found = session.get_node(guid);
    session.begin("remove");
    session.remove_object(guid);
    session.commit();
    const std::shared_ptr<TreeNode> removed = session.get_node(guid);
    const std::shared_ptr<TreeNode> orphaned = session.get_node(child_guid);
    session.undo();
    const std::shared_ptr<TreeNode> restored = session.get_node(guid);
    const std::shared_ptr<TreeNode> reattached = session.get_node(child_guid);
    const bool indexed = session.node_lookup.count(child_guid) && session.node_lookup.at(child_guid) == child;
    Tree tree("swapped");
    tree.add(std::make_shared<TreeNode>("root"));
    const std::shared_ptr<TreeNode> root = tree.root();
    const std::shared_ptr<TreeNode> swapped = std::make_shared<TreeNode>(guid);
    tree.add(swapped, root);
    session.tree = std::move(tree);
    const std::shared_ptr<TreeNode> searched = session.get_node(guid);
    session.reindex();

    MINI_CHECK(found == node);
    MINI_CHECK(removed == nullptr);
    MINI_CHECK(orphaned == child);
    MINI_CHECK(restored == node);
    MINI_CHECK(reattached == child && indexed);
    MINI_CHECK(searched == swapped);
    MINI_CHECK(session.node_lookup.at(guid) == swapped);
    MINI_CHECK(session.get_node("missing") == nullptr);
}

MINI_TEST("Session", "Remove Keeps Slot") {

    Session session;
    const std::shared_ptr<TreeNode> g = session.add_group("g");
    const std::shared_ptr<Point> a = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::shared_ptr<Point> b = std::make_shared<Point>(2.0, 0.0, 0.0);
    const std::shared_ptr<Point> c = std::make_shared<Point>(3.0, 0.0, 0.0);
    const std::string a_guid = a->guid();
    const std::string b_guid = b->guid();
    const std::string c_guid = c->guid();
    session.add_point(a, g);
    const std::shared_ptr<TreeNode> b_node = session.add_point(b, g);
    session.add_point(c, g);
    session.set_xform(b_guid, Xform::translation(0.0, 1.0, 0.0));
    const std::shared_ptr<Point> stored = session.objects.points->at(1);

    session.begin("remove");
    session.remove_object(b_guid);
    session.commit();
    const RemoveOp* record = std::get_if<RemoveOp>(&session.history.undo_stack[0].ops[0]);
    const bool pinned = record && record->tomb->node == b_node;

    MINI_CHECK(session.objects.points->size() == 2);
    MINI_CHECK(session.objects.points->number_of_slots() == 3);
    MINI_CHECK(session.objects.points->get_item(1) == stored);
    MINI_CHECK(session.lookup.count(b_guid) == 0);
    MINI_CHECK(session.xforms.count(b_guid) == 0);
    MINI_CHECK(!session.graph.has_node(b_guid));
    MINI_CHECK(session.get_node(b_guid) == nullptr);
    MINI_CHECK(pinned);

    session.undo();
    std::vector<std::string> names;

    for (const TreeNode* child : g->children())
        names.push_back(child->name);

    MINI_CHECK(session.objects.points->get_item(1) == stored);
    MINI_CHECK(session.objects.points->get_slot(b_guid) == 1);
    MINI_CHECK(session.order() == std::vector<std::string>({a_guid, b_guid, c_guid}));
    MINI_CHECK(names == std::vector<std::string>({a_guid, b_guid, c_guid}));
    MINI_CHECK(g->children()[1] == b_node.get());
    MINI_CHECK(b_node->at() == 1);
}

MINI_TEST("Session", "Redo Add Keeps Node") {

    Session session;
    const std::shared_ptr<Point> a = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::string a_guid = a->guid();

    session.begin("add");
    const std::shared_ptr<TreeNode> node = session.add_point(a);
    session.commit();
    const std::string node_guid = node->guid();
    const std::shared_ptr<Point> stored = session.objects.points->at(0);
    session.undo();
    const bool gone = session.get_node(a_guid) == nullptr && session.objects.points->empty();
    session.redo();
    const std::shared_ptr<TreeNode> found = session.get_node(a_guid);

    MINI_CHECK(gone);
    MINI_CHECK(found == node);
    MINI_CHECK(node->guid() == node_guid);
    MINI_CHECK(session.objects.points->at(0) == stored);
    MINI_CHECK(session.objects.points->get_slot(a_guid) == 0);
    MINI_CHECK(session.tree.root()->children().size() == 1);
}

MINI_TEST("Session", "Replace Shares Geometry") {

    Session session;
    const std::shared_ptr<Point> a = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::string a_guid = a->guid();
    session.add_point(a);
    const std::shared_ptr<Point> original = session.objects.points->at(0);
    const std::shared_ptr<Point> p2 = std::make_shared<Point>(9.0, 9.0, 9.0);
    p2->guid() = a_guid;

    session.begin("replace");
    session.replace(a_guid, p2);
    session.commit();
    const ReplaceOp* record = std::get_if<ReplaceOp>(&session.history.undo_stack[0].ops[0]);
    const bool shared = record && std::get<Geometry>(record->before) == Geometry(original)
        && std::get<Geometry>(record->after) == Geometry(p2);
    const bool swapped = session.objects.points->at(0) == p2;
    session.undo();
    const bool restored = session.objects.points->at(0) == original && session.objects.points->get_slot(a_guid) == 0;
    session.redo();

    MINI_CHECK(shared);
    MINI_CHECK(swapped);
    MINI_CHECK(restored);
    MINI_CHECK(session.objects.points->at(0) == p2);
    MINI_CHECK(std::get<std::shared_ptr<Point>>(session.lookup[a_guid]) == p2);
}

MINI_TEST("Session", "Replace Across Types") {

    Session session;
    const std::shared_ptr<Point> a = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::string a_guid = a->guid();
    const std::shared_ptr<TreeNode> node = session.add_point(a);
    const std::shared_ptr<Line> line = std::make_shared<Line>(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    line->name = "edge";

    session.begin("replace");
    const bool replaced = session.replace(a_guid, line);
    session.commit();
    const std::string label = session.graph.node_label(a_guid);
    const bool same_node = session.get_node(a_guid) == node;
    const std::pair<size_t, size_t> counts(session.objects.points->size(), session.objects.lines->size());
    session.undo();
    const std::pair<size_t, size_t> undone(session.objects.points->size(), session.objects.lines->size());
    const std::string restored = session.graph.node_label(a_guid);
    session.redo();

    MINI_CHECK(replaced);
    MINI_CHECK(counts.first == 0 && counts.second == 1);
    MINI_CHECK(same_node);
    MINI_CHECK(label == "line_edge");
    MINI_CHECK(undone.first == 1 && undone.second == 0);
    MINI_CHECK(restored == "point_my_point");
    MINI_CHECK(session.objects.lines->size() == 1);
    MINI_CHECK(session.objects.points->empty());
    MINI_CHECK(std::holds_alternative<std::shared_ptr<Line>>(session.lookup[a_guid]));
    MINI_CHECK(session.get_node(a_guid) == node);
}

MINI_TEST("Session", "Undo Restores Graph") {

    Session session;
    const std::shared_ptr<Point> a = std::make_shared<Point>(0.0, 0.0, 0.0);
    const std::shared_ptr<Point> b = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::shared_ptr<Point> c = std::make_shared<Point>(2.0, 0.0, 0.0);
    const std::string a_guid = a->guid();
    const std::string b_guid = b->guid();
    const std::string c_guid = c->guid();
    session.add_point(a);
    session.add_point(b);
    session.add_point(c);
    session.graph.set_vertex_attribute(a_guid, "mass", 2.0);
    session.add_edge(a_guid, b_guid, "joint");
    session.graph.set_edge_attribute({a_guid, b_guid}, "load", 1.5);
    session.add_edge(a_guid, c_guid, "contact");
    session.add_edge(b_guid, c_guid, "contact");
    const std::string before = session.graph.jsondump().dump();

    session.begin("remove");
    session.remove_object(a_guid);
    session.commit();
    const bool taken = !session.graph.has_node(a_guid) && session.graph.number_of_edges() == 1;
    session.undo();
    const std::string after = session.graph.jsondump().dump();
    session.redo();

    MINI_CHECK(taken);
    MINI_CHECK(before == after);
    MINI_CHECK(!session.graph.has_node(a_guid));
    MINI_CHECK(session.graph.has_edge({b_guid, c_guid}));
    MINI_CHECK(session.graph.number_of_vertices() == 2);

    const std::shared_ptr<TreeNode> c_node = session.get_node(c_guid);
    session.tree.remove(c_node);
    session.remove_object(c_guid);

    MINI_CHECK(!session.graph.has_node(c_guid));
}

MINI_TEST("Session", "Undo Restores Interactions") {

    Session session;
    const std::shared_ptr<Element> a = std::make_shared<Element>("a");
    const std::shared_ptr<Element> b = std::make_shared<Element>("b");
    const std::shared_ptr<Element> c = std::make_shared<Element>("c");
    session.add_element(a);
    session.add_element(b);
    session.add_element(c);
    const std::string glue = session.add_interaction(a, b, std::make_shared<NamedInteraction>("glue"))->guid();
    const std::string nail = session.add_interaction(a, c, std::make_shared<NamedInteraction>("nail"))->guid();
    const std::string ab = session.graph.edges[a->guid()][b->guid()].guid();
    const std::string ac = session.graph.edges[a->guid()][c->guid()].guid();

    session.begin("remove");
    session.remove_object(a->guid());
    session.commit();
    const bool parked = session.interactions.empty();
    session.undo();
    session.redo();
    session.undo();

    MINI_CHECK(parked);
    MINI_CHECK(session.interactions.size() == 2);
    MINI_CHECK(session.graph.edges[a->guid()][b->guid()].guid() == ab);
    MINI_CHECK(session.graph.edges[a->guid()][c->guid()].guid() == ac);
    MINI_CHECK(session.get_interaction(a, b)[0]->guid() == glue);
    MINI_CHECK(session.get_interaction(a, c)[0]->guid() == nail);
    MINI_CHECK(session.get_interaction(a, c)[0]->name == "nail");
}

MINI_TEST("Session", "Dead Guid Reused") {

    Session session;
    const std::shared_ptr<Point> x = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::string x_guid = x->guid();
    session.add_point(x);
    const std::shared_ptr<Point> again = std::make_shared<Point>(9.0, 0.0, 0.0);
    again->guid() = x_guid;

    session.begin("reuse");
    session.remove_object(x_guid);
    session.add_point(again);
    session.commit();
    const double first = (*std::get<std::shared_ptr<Point>>(session.lookup[x_guid]))[0];
    const std::pair<bool, std::optional<size_t>> slots(session.objects.points->is_dead(0), session.objects.points->get_slot(x_guid));
    const size_t count = session.objects.points->size();
    session.undo();
    const double second = (*std::get<std::shared_ptr<Point>>(session.lookup[x_guid]))[0];
    const std::pair<bool, std::optional<size_t>> undone(session.objects.points->is_dead(1), session.objects.points->get_slot(x_guid));
    const size_t still = session.objects.points->size();
    session.redo();

    MINI_CHECK(TOLERANCE.is_close(first, 9.0));
    MINI_CHECK(slots.first && slots.second == 1);
    MINI_CHECK(count == 1);
    MINI_CHECK(TOLERANCE.is_close(second, 1.0));
    MINI_CHECK(undone.first && undone.second == 0);
    MINI_CHECK(still == 1);
    MINI_CHECK(session.objects.points->get_slot(x_guid) == 1);
    MINI_CHECK(session.objects.points->size() == 1);
}

MINI_TEST("Session", "Remove Keeps Lookup Edit") {

    Session session;
    const std::shared_ptr<Point> a = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::string a_guid = a->guid();
    session.add_point(a);
    const std::shared_ptr<Point> edited = std::make_shared<Point>(7.0, 0.0, 0.0);
    edited->guid() = a_guid;
    session.lookup[a_guid] = edited;

    session.begin("remove");
    session.remove_object(a_guid);
    session.commit();
    session.undo();
    const std::string json = session.jsondump().dump();

    MINI_CHECK(TOLERANCE.is_close((*std::get<std::shared_ptr<Point>>(session.lookup[a_guid]))[0], 7.0));
    MINI_CHECK(TOLERANCE.is_close((*session.objects.points->at(0))[0], 7.0));
    MINI_CHECK(json.find("7.0") != std::string::npos);
}

MINI_TEST("Session", "Tree Ops") {

    Session session;
    session.add_group("A");
    const std::shared_ptr<TreeNode> root = session.tree.root();
    std::vector<std::string> snapshots = {session.tree.str()};

    session.begin("group");
    const std::shared_ptr<TreeNode> node = session.add_group("L");
    session.commit();
    snapshots.push_back(session.tree.str());

    session.begin("rename");
    const bool renamed = session.rename_node(node, "M");
    session.commit();
    snapshots.push_back(session.tree.str());

    session.begin("colour");
    const bool coloured = session.set_node_color(node, Color(1.0, 0.0, 0.0, 1.0));
    session.commit();
    snapshots.push_back(session.tree.str());

    session.begin("remove");
    const bool removed = session.remove_group(node);
    session.commit();
    const bool dead = node->is_dead() && root->children().size() == 1;
    std::vector<bool> restored;

    for (int i = 3; i >= 0; --i) {
        session.undo();
        restored.push_back(session.tree.str() == snapshots[i]);
    }

    const bool absent = node->is_dead() && session.tree.get_node_by_name("L") == nullptr;
    std::vector<bool> redone;

    for (int i = 1; i <= 4; ++i) {
        session.redo();
        redone.push_back(i == 4 || session.tree.str() == snapshots[i]);
    }

    MINI_CHECK(renamed && coloured && removed);
    MINI_CHECK(dead);
    MINI_CHECK(restored == std::vector<bool>({true, true, true, true}));
    MINI_CHECK(absent);
    MINI_CHECK(redone == std::vector<bool>({true, true, true, true}));
    MINI_CHECK(node->is_dead());
    MINI_CHECK(node->name == "M");
    MINI_CHECK(node->at() == 1);
    MINI_CHECK(node->color.has_value());
    MINI_CHECK(session.tree.root() == root);
    MINI_CHECK(std::holds_alternative<TreeOp>(session.history.undo_stack[3].ops[0]));

    const std::shared_ptr<Point> point = std::make_shared<Point>(0.0, 0.0, 0.0);
    const std::string guid = point->guid();
    const std::shared_ptr<TreeNode> held = session.add_point(point);
    session.tree.remove(held);
    session.set_xform(guid, Xform::translation(1.0, 0.0, 0.0));
    session.begin("adopt");
    session.add(std::make_shared<TreeNode>(guid));
    session.commit();
    session.undo();

    MINI_CHECK(session.get_node(guid) == nullptr);
    MINI_CHECK(session.xforms.count(guid) == 1);
}

MINI_TEST("Session", "Move Node") {

    Session session;
    const std::shared_ptr<TreeNode> g1 = session.add_group("g1");
    const std::shared_ptr<TreeNode> g2 = session.add_group("g2");
    session.add_point(std::make_shared<Point>(0.0, 0.0, 0.0), g1);
    const std::shared_ptr<TreeNode> x = session.add_point(std::make_shared<Point>(1.0, 0.0, 0.0), g1);
    const std::shared_ptr<TreeNode> y = session.add_point(std::make_shared<Point>(2.0, 0.0, 0.0), x);
    const size_t count = session.tree.nodes().size();

    session.begin("move");
    session.add(x, g2);
    session.commit();
    const bool queued = g1->is_queued();
    const bool moved = g1->children().size() == 1 && !g2->children().empty() && g2->children().back() == x.get()
        && y->parent() == x;
    const size_t walked = session.tree.nodes().size();
    session.undo();
    const bool back = g1->children().size() > 1 && g1->children()[1] == x.get() && g2->children().empty();
    const std::string text = session.tree.str();
    session.redo();

    MINI_CHECK(moved);
    MINI_CHECK(queued);
    MINI_CHECK(g2->is_queued());
    MINI_CHECK(walked == count);
    MINI_CHECK(back);
    MINI_CHECK(session.tree.nodes().size() == count);
    MINI_CHECK(text.find("TreeNode(, ") == std::string::npos);
    MINI_CHECK(g2->children().size() == 1);
    MINI_CHECK(g1->children().size() == 1);
    MINI_CHECK(x->parent() == g2);
}

MINI_TEST("Session", "Deleted Parent Orphans Children") {

    Session session;
    const std::shared_ptr<Element> element = std::make_shared<Element>("E");
    const std::string e_guid = element->guid();
    const std::shared_ptr<TreeNode> e_node = session.add_element(element);
    const std::shared_ptr<TreeNode> attributes = std::make_shared<TreeNode>("attributes");
    session.add(attributes, e_node);
    const std::shared_ptr<Polyline> q = std::make_shared<Polyline>(std::vector<Point>{Point(0, 0, 0), Point(1, 0, 0)});
    const std::string q_guid = q->guid();
    const std::shared_ptr<TreeNode> q_node = session.add_polyline(q, attributes);
    session.set_xform(e_guid, Xform::translation(1.0, 0.0, 0.0));
    session.set_xform(q_guid, Xform::translation(0.0, 1.0, 0.0));
    const Xform composed = session.world_xform(q_guid);

    session.begin("remove");
    session.remove_object(e_guid);
    session.commit();
    std::vector<std::string> names;

    for (const std::shared_ptr<TreeNode>& node : session.tree.nodes())
        names.push_back(node->name);

    MINI_CHECK(session.lookup.count(q_guid) == 1);
    MINI_CHECK(session.get_node(q_guid) == q_node);
    MINI_CHECK(q_node->parent() == attributes);
    MINI_CHECK(attributes->parent() == e_node);
    MINI_CHECK(e_node->parent() == nullptr);
    MINI_CHECK(session.world_xform(q_guid) == session.xform(q_guid));
    MINI_CHECK(session.world_xforms().count(q_guid) == 1);
    MINI_CHECK(names == std::vector<std::string>({"my_session"}));

    session.undo();

    MINI_CHECK(session.world_xform(q_guid) == composed);
    MINI_CHECK(session.tree.nodes().size() == 4);
}

MINI_TEST("Session", "Copy Drops Dead") {

    Session session;
    const std::shared_ptr<TreeNode> g = session.add_group("g");
    const std::shared_ptr<Point> a = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::shared_ptr<Point> b = std::make_shared<Point>(2.0, 0.0, 0.0);
    const std::string b_guid = b->guid();
    session.add_point(a, g);
    session.add_point(b, g);

    session.begin("remove");
    session.remove_object(b_guid);
    session.commit();
    const Session copy = session;

    MINI_CHECK(copy.objects.points->number_of_slots() == copy.objects.points->size());
    MINI_CHECK(copy.objects.points->number_of_dead() == 0);
    MINI_CHECK(copy.objects.points->size() == 1);
    MINI_CHECK(copy.history.depth() == 0);
    MINI_CHECK(copy.order() == session.order());
    MINI_CHECK(copy.tree.str() == session.tree.str());
    MINI_CHECK(copy.graph.number_of_vertices() == 1);
    MINI_CHECK(session.undo());
    MINI_CHECK(session.objects.points->size() == 2);
    MINI_CHECK(copy.objects.points->size() == 1);
}

MINI_TEST("Session", "Live Views") {

    Session session;
    const std::shared_ptr<TreeNode> g = session.add_group("gone_group");
    const std::shared_ptr<TreeNode> kept = session.add_group("kept");
    const std::string definition = session.add_definition(create_box(Point(0, 0, 0), 2.0));
    const std::shared_ptr<Point> point = std::make_shared<Point>(0.0, 0.0, 0.0);
    const std::string point_guid = point->guid();
    const std::shared_ptr<Mesh> mesh = create_box(Point(5.0, 0.0, 0.0), 2.0);
    const std::string mesh_guid = mesh->guid();
    const std::shared_ptr<InstanceRef> instance = std::make_shared<InstanceRef>(definition, Xform::identity());
    const std::string instance_guid = instance->guid();
    session.add_point(point, g);
    session.add_mesh(mesh, g);
    session.add_instance(instance, Xform::translation(10.0, 0.0, 0.0), g);
    session.add_point(std::make_shared<Point>(20.0, 0.0, 0.0), kept);
    session.set_xform(point_guid, Xform::translation(0.0, 1.0, 0.0));
    session.set_xform(mesh_guid, Xform::translation(0.0, 1.0, 0.0));
    session.set_xform("gone_group", Xform::translation(0.0, 0.0, 1.0));

    session.begin("remove");
    session.remove_object(point_guid);
    session.remove_object(mesh_guid);
    session.remove_object(instance_guid);
    session.remove_group(g);
    session.commit();
    const std::vector<std::string> gone = {point_guid, mesh_guid, instance_guid, "gone_group"};
    const std::vector<std::string> order = session.order();
    const std::unordered_map<std::string, Xform> world = session.world_xforms();
    std::vector<std::string> groups;

    for (const TreeNode* child : session.tree.root()->children())
        groups.push_back(child->name);

    const Objects geometry = session.get_geometry();
    const std::vector<std::pair<std::string, std::string>> collisions = session.get_collisions();
    const std::vector<Session::RayHit> hits = session.ray_cast(Point(5.0, 0.0, -10.0), Vector(0.0, 0.0, 1.0), 0.01);
    const std::string json = session.jsondump().dump();
    const std::string text = session.str() + session.repr();
    bool ordered = true;
    bool placed = true;
    bool dumped = true;
    bool printed = true;

    for (const std::string& name : gone) {
        ordered = ordered && std::find(order.begin(), order.end(), name) == order.end();
        placed = placed && world.count(name) == 0;
        dumped = dumped && json.find(name) == std::string::npos;
        printed = printed && text.find(name) == std::string::npos;
    }

    MINI_CHECK(ordered);
    MINI_CHECK(placed);
    MINI_CHECK(session.select_by_type<Point>().size() == 1);
    MINI_CHECK(groups == std::vector<std::string>({"kept"}));
    MINI_CHECK(geometry.points->size() == 1 && geometry.meshes->empty());
    MINI_CHECK(session.instances_of(definition).empty());
    MINI_CHECK(collisions.empty());
    MINI_CHECK(hits.empty());
    MINI_CHECK(dumped);
    MINI_CHECK(printed);
    MINI_CHECK(session.undo());
    MINI_CHECK(session.order().size() == 3);
}

MINI_TEST("Session", "Unrecorded Remove") {

    Session session;
    const std::shared_ptr<Point> a = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::string a_guid = a->guid();
    session.add_point(a);
    session.add_point(std::make_shared<Point>(2.0, 0.0, 0.0));
    const bool removed = session.remove_object(a_guid);

    MINI_CHECK(removed);
    MINI_CHECK(session.lookup.count(a_guid) == 0);
    MINI_CHECK(session.order().size() == 1);
    MINI_CHECK(session.tree.nodes().size() == 2);
    MINI_CHECK(session.objects.points->number_of_dead() == 1);
    MINI_CHECK(session.objects.points->get_tomb(0) == nullptr);
    MINI_CHECK(session.history.dropped == 1);
    MINI_CHECK(!session.undo());
}

MINI_TEST("Session", "Remove Twin Keeps Slot") {

    Session session;
    const std::shared_ptr<Point> x = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::string guid = x->guid();
    const std::shared_ptr<Point> y = std::make_shared<Point>(2.0, 0.0, 0.0);
    y->guid() = guid;
    session.begin("add");
    session.add_point(x);
    session.commit();
    session.undo();
    session.add_point(y);
    session.set_xform(guid, Xform::translation(0.0, 1.0, 0.0));
    session.redo();
    session.undo();

    MINI_CHECK(session.objects.points->get_slot(guid) == 1);
    MINI_CHECK(session.objects.points->is_dead(0));
    MINI_CHECK(session.objects.points->size() == 1);
    MINI_CHECK(session.lookup.count(guid) == 1);
    MINI_CHECK(session.xforms.count(guid) == 1);
    MINI_CHECK(session.graph.has_node(guid));

    const bool removed = session.remove_object(guid);
    const std::string bytes = session.pb_dumps();
    const Session loaded = Session::pb_loads(bytes);

    MINI_CHECK(removed);
    MINI_CHECK(session.lookup.count(guid) == 0);
    MINI_CHECK(session.objects.points->empty());
    MINI_CHECK(loaded.objects.points->empty());
    MINI_CHECK(loaded.lookup.empty());
    MINI_CHECK(loaded.graph.number_of_vertices() == 0);
}

MINI_TEST("Session", "Redo Twin Keeps Slot") {

    Session session;
    const std::shared_ptr<Point> x = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::string guid = x->guid();
    const std::shared_ptr<Point> y = std::make_shared<Point>(2.0, 0.0, 0.0);
    y->guid() = guid;
    session.begin("add");
    session.add_point(x);
    session.commit();
    session.undo();
    session.add_point(y);
    session.redo();
    double held = 0.0;
    bool live_node = false;

    if (const std::shared_ptr<Point>* point = std::get_if<std::shared_ptr<Point>>(&session.lookup[guid]))
        held = (**point)[0];

    if (const std::shared_ptr<TreeNode> node = session.get_node(guid))
        live_node = !node->is_dead();

    MINI_CHECK(session.objects.points->size() == 1);
    MINI_CHECK(session.objects.points->get_slot(guid) == 1);
    MINI_CHECK(session.objects.points->is_dead(0));
    MINI_CHECK(held == 2.0);
    MINI_CHECK(session.graph.has_node(guid));
    MINI_CHECK(live_node);

    session.undo();
    session.redo();

    MINI_CHECK(session.objects.points->size() == 1);
    MINI_CHECK(session.objects.points->get_slot(guid) == 1);

    const bool removed = session.remove_object(guid);
    const bool undone = session.undo();
    const std::string bytes = session.pb_dumps();
    const Session loaded = Session::pb_loads(bytes);

    MINI_CHECK(removed);
    MINI_CHECK(undone);
    MINI_CHECK(session.lookup.count(guid) == 0);
    MINI_CHECK(!session.graph.has_node(guid));
    MINI_CHECK(session.objects.points->empty());
    MINI_CHECK(loaded.objects.points->empty());
    MINI_CHECK(loaded.lookup.empty());
    MINI_CHECK(loaded.tree.nodes().size() == 1);
}

MINI_TEST("Session", "Cross Type Twin") {

    Session session;
    const std::shared_ptr<Point> x = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::string guid = x->guid();
    const std::shared_ptr<Line> y = std::make_shared<Line>(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    y->guid() = guid;
    session.begin("add");
    session.add_point(x);
    session.commit();
    session.undo();
    session.add_line(y);
    session.set_xform(guid, Xform::translation(0.0, 1.0, 0.0));
    session.redo();
    const std::tuple<bool, std::optional<size_t>, bool> redone = {
        session.objects.points->is_dead(0),
        session.objects.lines->get_slot(guid),
        std::holds_alternative<std::shared_ptr<Line>>(session.lookup[guid]),
    };
    session.undo();
    const std::tuple<std::optional<size_t>, bool, bool> undone = {
        session.objects.lines->get_slot(guid),
        session.xforms.count(guid) == 1,
        session.graph.has_node(guid),
    };
    session.redo();

    MINI_CHECK(redone == std::make_tuple(true, std::optional<size_t>(0), true));
    MINI_CHECK(undone == std::make_tuple(std::optional<size_t>(0), true, true));
    MINI_CHECK(session.objects.points->empty());
    MINI_CHECK(session.objects.lines->size() == 1);
    MINI_CHECK(session.order() == std::vector<std::string>({guid}));

    const bool removed = session.remove_object(guid);
    const bool unremoved = session.undo();
    const std::string bytes = session.pb_dumps();
    const Session loaded = Session::pb_loads(bytes);

    MINI_CHECK(removed);
    MINI_CHECK(unremoved);
    MINI_CHECK(session.lookup.count(guid) == 0);
    MINI_CHECK(session.objects.lines->empty());
    MINI_CHECK(session.objects.points->empty());
    MINI_CHECK(loaded.objects.lines->empty());
    MINI_CHECK(loaded.objects.points->empty());
    MINI_CHECK(loaded.graph.number_of_vertices() == 0);
    MINI_CHECK(loaded.tree.nodes().size() == 1);
}

MINI_TEST("Session", "Add Live Guid Refused") {

    Session session;
    const std::shared_ptr<Point> x = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::string guid = x->guid();
    const std::shared_ptr<TreeNode> node = session.add_point(x);
    const std::string definition = session.add_definition(std::make_shared<Point>(0.0, 0.0, 0.0));
    const uint64_t revision = session.revision;
    const std::shared_ptr<Point> point = std::make_shared<Point>(2.0, 0.0, 0.0);
    point->guid() = guid;
    const std::shared_ptr<Line> line = std::make_shared<Line>(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    line->guid() = guid;
    const std::shared_ptr<Polyline> polyline = std::make_shared<Polyline>(
        std::vector<Point>({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0)})
    );
    polyline->guid() = guid;
    Component component;
    component.guid() = guid;
    const std::shared_ptr<InstanceRef> instance = std::make_shared<InstanceRef>(definition, Xform::identity());
    instance->guid() = guid;
    const bool same_point = session.add_point(point) == node;
    const bool same_line = session.add_line(line) == node;
    const bool no_polyline = session.add_polyline(polyline) == nullptr;
    const bool same_component = session.add_component(component) == node;
    const bool no_instance = session.add_instance(instance, Xform::identity()) == nullptr;
    double held = 0.0;

    if (const std::shared_ptr<Point>* stored = std::get_if<std::shared_ptr<Point>>(&session.lookup[guid]))
        held = (**stored)[0];

    MINI_CHECK(same_point && same_line && same_component);
    MINI_CHECK(no_polyline && no_instance);
    MINI_CHECK(held == 1.0);
    MINI_CHECK(session.objects.points->size() == 1);
    MINI_CHECK(session.objects.lines->empty());
    MINI_CHECK(session.objects.polylines->empty());
    MINI_CHECK(session.objects.components->empty());
    MINI_CHECK(session.objects.instances->empty());
    MINI_CHECK(session.revision == revision);
    MINI_CHECK(session.tree.nodes().size() == 2);

    const std::shared_ptr<Point> again = std::make_shared<Point>(3.0, 0.0, 0.0);
    again->guid() = guid;
    session.begin("twin");
    session.add_point(again);
    session.commit();

    MINI_CHECK(session.history.depth() == 0);
    MINI_CHECK(!session.undo());
    MINI_CHECK(session.objects.points->size() == 1);
}

MINI_TEST("Session", "Twin Skips Recorded Edits") {

    Session session;
    const std::shared_ptr<Point> x = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::string guid = x->guid();
    const std::shared_ptr<Line> y = std::make_shared<Line>(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    y->guid() = guid;
    const std::shared_ptr<Point> moved = std::make_shared<Point>(5.0, 0.0, 0.0);
    moved->guid() = guid;
    session.begin("edit");
    session.add_point(x);
    session.set_xform(guid, Xform::translation(0.0, 0.0, 1.0));
    session.replace(guid, moved);
    session.commit();
    session.undo();
    session.add_line(y);
    session.set_xform(guid, Xform::translation(0.0, 1.0, 0.0));
    const Xform placed = Xform::translation(0.0, 1.0, 0.0);
    session.redo();
    const std::pair<bool, bool> redone = {
        std::holds_alternative<std::shared_ptr<Line>>(session.lookup[guid]),
        session.xform(guid) == placed,
    };
    session.undo();
    const std::pair<bool, bool> undone = {
        std::holds_alternative<std::shared_ptr<Line>>(session.lookup[guid]),
        session.xform(guid) == placed,
    };

    MINI_CHECK(redone == std::make_pair(true, true));
    MINI_CHECK(undone == std::make_pair(true, true));
    MINI_CHECK(session.objects.lines->size() == 1);
    MINI_CHECK(session.objects.points->empty());

    const std::string definition = session.add_definition(std::make_shared<Point>(0.0, 0.0, 0.0));
    const std::shared_ptr<InstanceRef> instance = std::make_shared<InstanceRef>(definition, Xform::identity());
    const std::string instance_guid = instance->guid();
    const std::shared_ptr<Point> twin = std::make_shared<Point>(2.0, 0.0, 0.0);
    twin->guid() = instance_guid;
    session.begin("place");
    session.add_instance(instance, Xform::translation(3.0, 0.0, 0.0));
    session.commit();
    session.undo();
    session.add_point(twin);
    session.redo();
    const bool twin_placed = session.xforms.count(instance_guid) == 1;
    session.undo();

    MINI_CHECK(!twin_placed);
    MINI_CHECK(session.xforms.count(instance_guid) == 0);
    MINI_CHECK(session.lookup.count(instance_guid) == 1);
    MINI_CHECK(session.objects.instances->empty());

    const std::shared_ptr<Point> held = std::make_shared<Point>(0.0, 0.0, 0.0);
    const std::string held_guid = held->guid();
    const std::shared_ptr<Point> swapped = std::make_shared<Point>(9.0, 0.0, 0.0);
    swapped->guid() = held_guid;
    const std::shared_ptr<Point> taker = std::make_shared<Point>(7.0, 0.0, 0.0);
    taker->guid() = held_guid;
    session.begin("define");
    session.add_definition(held);
    session.commit();
    session.begin("swap");
    session.replace_definition(held_guid, swapped);
    session.commit();
    session.undo();
    session.undo();
    session.add_point(taker);
    session.redo();
    session.redo();
    bool taker_redone = false;
    bool taker_undone = false;

    if (const std::shared_ptr<Point>* point = std::get_if<std::shared_ptr<Point>>(&session.lookup[held_guid]))
        taker_redone = (**point)[0] == 7.0;

    session.undo();

    if (const std::shared_ptr<Point>* point = std::get_if<std::shared_ptr<Point>>(&session.lookup[held_guid]))
        taker_undone = (**point)[0] == 7.0;

    MINI_CHECK(taker_redone);
    MINI_CHECK(taker_undone);
    MINI_CHECK(session.definition_lookup.count(held_guid) == 0);
}

MINI_TEST("Session", "Definition Guid Is Live") {

    Session session;
    const std::string definition = session.add_definition(std::make_shared<Point>(0.0, 0.0, 0.0));
    const uint64_t revision = session.revision;
    const std::shared_ptr<Point> point = std::make_shared<Point>(1.0, 0.0, 0.0);
    point->guid() = definition;
    const std::shared_ptr<TreeNode> node = session.add_point(point);

    MINI_CHECK(node->parent() == nullptr);
    MINI_CHECK(session.objects.points->empty());
    MINI_CHECK(session.lookup.count(definition) == 0);
    MINI_CHECK(session.revision == revision);

    const std::shared_ptr<Point> again = std::make_shared<Point>(2.0, 0.0, 0.0);
    again->guid() = definition;
    session.begin("define");
    session.remove_definition(definition);
    session.commit();
    session.add_point(again);
    session.undo();

    MINI_CHECK(session.lookup.count(definition) == 1);
    MINI_CHECK(session.definition_lookup.count(definition) == 0);
    MINI_CHECK(session.definitions.points->is_dead(0));

    const std::shared_ptr<Point> x = std::make_shared<Point>(3.0, 0.0, 0.0);
    const std::string guid = x->guid();
    session.begin("add");
    session.add_point(x);
    session.commit();
    session.undo();
    const std::string defined = session.add_definition(std::make_shared<Point>(4.0, 0.0, 0.0));
    const std::shared_ptr<Point> shared = std::make_shared<Point>(5.0, 0.0, 0.0);
    shared->guid() = guid;
    const std::string taken = session.add_definition(shared);
    session.redo();

    MINI_CHECK(defined != guid);
    MINI_CHECK(taken == guid);
    MINI_CHECK(session.lookup.count(guid) == 0);
    MINI_CHECK(!session.objects.points->get_slot(guid));
    MINI_CHECK(session.objects.points->size() == 1);
    MINI_CHECK(session.definition_lookup.count(guid) == 1);
}

MINI_TEST("Session", "Purge Clears History") {

    Session session;
    const std::shared_ptr<Point> a = std::make_shared<Point>(0.0, 0.0, 0.0);
    const std::shared_ptr<Point> b = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::shared_ptr<Point> c = std::make_shared<Point>(2.0, 0.0, 0.0);
    const std::string a_guid = a->guid();
    const std::string b_guid = b->guid();
    const std::string c_guid = c->guid();
    session.add_point(a);
    session.add_point(b);
    session.add_point(c);
    session.begin("remove");
    session.remove_object(b_guid);
    session.commit();
    session.purge();
    const bool undone = session.undo();
    std::vector<int> indices;

    for (const Vertex& vertex : session.graph.get_vertices())
        indices.push_back(vertex.index);

    std::sort(indices.begin(), indices.end());

    MINI_CHECK(!undone);
    MINI_CHECK(session.history.depth() == 0);
    MINI_CHECK(session.order() == std::vector<std::string>({a_guid, c_guid}));
    MINI_CHECK(indices == std::vector<int>({0, 1}));
    MINI_CHECK(session.objects.points->number_of_slots() == 2);
    MINI_CHECK(session.number_of_dead() == 0);
    MINI_CHECK(session.tree.nodes().size() == 3);
}

MINI_TEST("Session", "Checkpoint Tags") {

    session_proto::Session objects;
    objects.mutable_objects();
    session_proto::Session tree;
    tree.mutable_tree();
    session_proto::Session graph;
    graph.mutable_graph();
    session_proto::Session definitions;
    definitions.mutable_definitions();
    session_proto::Tree root;
    root.mutable_root();
    session_proto::TreeNode children;
    children.add_children();
    std::array<session_proto::Objects, 13> lists;
    lists[0].add_points();
    lists[1].add_lines();
    lists[2].add_planes();
    lists[3].add_bboxes();
    lists[4].add_polylines();
    lists[5].add_pointclouds();
    lists[6].add_meshes();
    lists[7].add_nurbscurves();
    lists[8].add_nurbssurfaces();
    lists[9].add_breps();
    lists[10].add_elements();
    lists[11].add_components();
    lists[12].add_instances();
    std::vector<std::string> messages = {
        objects.SerializeAsString(),
        tree.SerializeAsString(),
        graph.SerializeAsString(),
        definitions.SerializeAsString(),
        root.SerializeAsString(),
        children.SerializeAsString(),
    };

    for (const session_proto::Objects& list : lists)
        messages.push_back(list.SerializeAsString());

    std::vector<int> fields;

    for (const std::string& bytes : messages) {
        google::protobuf::io::CodedInputStream input(reinterpret_cast<const uint8_t*>(bytes.data()), static_cast<int>(bytes.size()));
        fields.push_back(static_cast<int>(input.ReadTag() >> 3));
    }

    const std::array<int, 7> sections = {0, fields[0], fields[1], fields[2], 0, fields[3], 0};
    std::array<int, 13> tags = {};

    for (size_t i = 0; i < tags.size(); ++i)
        tags[i] = fields[6 + i];

    MINI_CHECK(TAGS.sections == sections);
    MINI_CHECK(TAGS.root == fields[4]);
    MINI_CHECK(TAGS.children == fields[5]);
    MINI_CHECK(TAGS.lists == tags);
}

MINI_TEST("Session", "Checkpoint After Purge Steps") {

    const size_t n = 40000;
    const size_t bulk = 20000;
    Session session;
    const std::shared_ptr<TreeNode> group = session.add_group("flat");
    std::vector<std::string> guids;

    for (size_t i = 0; i < n; i++)
        guids.push_back(session.add_point(std::make_shared<Point>(static_cast<double>(i), 0.0, 0.0), group)->name);

    session.begin("remove");

    for (size_t i = 0; i < bulk; i++)
        session.remove_object(guids[i]);

    session.commit();

    for (int step = 0; step < CAPACITY; step++) {
        session.begin("move");
        session.set_xform(guids[n - 1], Xform::translation(static_cast<double>(step), 0.0, 0.0));
        session.commit();
    }

    int steps = 0;

    while (session.purge_step(PURGE_WORK))
        steps++;

    std::optional<std::string> bytes;

    while (!bytes)
        bytes = session.checkpoint(PURGE_WORK);

    const Session loaded = Session::pb_loads(*bytes);
    session_proto::Session parsed;
    MINI_CHECK(parsed.ParseFromString(*bytes));

    MINI_CHECK(steps > 1);
    MINI_CHECK(loaded.objects.points->size() == n - bulk);
    MINI_CHECK(google::protobuf::util::MessageDifferencer::Equals(parsed, session.to_proto()));
    MINI_CHECK(session.number_of_dead() == 0);
    MINI_CHECK(session.history.depth() == CAPACITY);
}

MINI_TEST("Session", "Steady State Bounds") {

    const size_t n = 2000;
    const size_t cycles = 1000;
    Session session;
    const std::shared_ptr<TreeNode> group = session.add_group("flat");
    std::vector<std::string> guids;

    for (size_t i = 0; i < n; i++)
        guids.push_back(session.add_point(std::make_shared<Point>(static_cast<double>(i), 0.0, 0.0), group)->name);

    for (size_t i = 0; i < cycles; i++) {
        session.begin("remove");
        session.remove_object(guids[i]);
        session.commit();
        session.undo();
        session.redo();
        const double x = static_cast<double>(session.objects.points->size());
        session.begin("add");
        session.add_point(std::make_shared<Point>(x, 1.0, 0.0), group);
        session.commit();
        session.purge_step(PURGE_WORK);
    }

    const size_t bound = 2 * CAPACITY + 2 * (n / PURGE_WORK + 1);
    const Collection<std::shared_ptr<Point>>& points = *session.objects.points;

    MINI_CHECK(points.size() == n);
    MINI_CHECK(session.history.bytes <= session.history.budget);
    MINI_CHECK(session.number_of_dead() <= bound);
    MINI_CHECK(points.number_of_slots() <= points.size() + bound);
    MINI_CHECK(group->children().size() <= points.size() + bound);
}

MINI_TEST("Session", "History Budget Bounds") {

    const size_t side = 30;
    std::vector<Point> vertices;
    std::vector<std::vector<size_t>> faces;

    for (size_t at = 0; at < side * side; at++)
        vertices.push_back(Point(static_cast<double>(at / side), static_cast<double>(at % side), 0.0));

    for (size_t cell = 0; cell < (side - 1) * (side - 1); cell++) {
        const size_t at = cell / (side - 1) * side + cell % (side - 1);
        faces.push_back({at, at + side, at + side + 1, at + 1});
    }

    Session session;
    session.history.budget = 4 << 20;
    std::vector<std::string> guids;

    for (int i = 0; i < 200; i++) {
        const std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>(Mesh::from_vertices_and_faces(vertices, faces));
        guids.push_back(mesh->guid());
        session.add_mesh(mesh);
    }

    bool bounded = true;

    for (const std::string& guid : guids) {
        session.begin("remove");
        session.remove_object(guid);
        session.commit();
        const size_t newest = session.history.undo_stack[session.history.depth() - 1].bytes;
        bounded &= session.history.bytes <= session.history.budget + newest;
    }

    MINI_CHECK(bounded);
    MINI_CHECK(session.history.depth() < CAPACITY);
    MINI_CHECK(session.history.depth() > 1);
    MINI_CHECK(session.objects.meshes->empty());
}

MINI_TEST("Session", "Purge Keeps Replaced Tomb") {

    Session session;
    const std::string definition = session.add_definition(std::make_shared<Point>(1.0, 2.0, 3.0));
    const std::shared_ptr<InstanceRef> first = std::make_shared<InstanceRef>(definition, Xform::identity());
    const std::shared_ptr<InstanceRef> second = std::make_shared<InstanceRef>(definition, Xform::identity());
    session.add_instance(first);
    session.begin("add");
    session.add_instance(second);
    session.commit();
    session.begin("explode");
    session.explode(second->guid());
    session.commit();
    session.remove_object(first->guid());

    while (session.purge_step(PURGE_WORK)) {}

    const size_t slots = session.objects.instances->number_of_slots();
    const bool unexploded = session.undo();
    const size_t instances = session.objects.instances->size();
    const bool unadded = session.undo();

    MINI_CHECK(slots == 1);
    MINI_CHECK(unexploded);
    MINI_CHECK(instances == 1);
    MINI_CHECK(unadded);
    MINI_CHECK(session.objects.instances->empty());
    MINI_CHECK(session.objects.points->empty());
    MINI_CHECK(session.instance_lookup.count(second->guid()) == 0);
    MINI_CHECK(session.redo());
    MINI_CHECK(session.redo());
    MINI_CHECK(session.objects.points->size() == 1);
}

MINI_TEST("Session", "Checkpoint Twin Xform") {

    Session session;
    const std::string definition = session.add_definition(std::make_shared<Point>(1.0, 2.0, 3.0));
    const std::shared_ptr<Point> x = std::make_shared<Point>(0.0, 0.0, 0.0);
    const std::shared_ptr<Point> y = std::make_shared<Point>(1.0, 0.0, 0.0);
    y->guid() = x->guid();
    const std::shared_ptr<InstanceRef> instance = std::make_shared<InstanceRef>(definition, Xform::identity());
    session.add_point(x);
    session.add_point(y);
    session.add_instance(instance, Xform::translation(0.0, 1.0, 0.0));
    session.set_xform(x->guid(), Xform::translation(1.0, 0.0, 0.0));
    std::optional<std::string> data = session.checkpoint(1);

    while (!data)
        data = session.checkpoint(1);

    const Session loaded = Session::pb_loads(*data);
    session_proto::Session parsed;
    MINI_CHECK(parsed.ParseFromString(*data));

    MINI_CHECK(google::protobuf::util::MessageDifferencer::Equals(parsed, session.to_proto()));
    MINI_CHECK(loaded.xform(x->guid()) == session.xform(x->guid()));
    MINI_CHECK(loaded.xform(instance->guid()) == session.xform(instance->guid()));
}

MINI_TEST("Session", "Checkpoint Keeps Replaced Definition") {

    Session session;
    const std::shared_ptr<Point> a = std::make_shared<Point>(0.0, 0.0, 0.0);
    const std::shared_ptr<Point> b = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::shared_ptr<Point> c = std::make_shared<Point>(2.0, 0.0, 0.0);
    const std::string guid = b->guid();
    c->guid() = guid;
    session.add_definition(a);
    session.add_definition(b);
    session.remove_definition(a->guid());
    session.begin("swap");
    session.replace_definition(guid, c);
    session.commit();
    std::optional<std::string> data = session.checkpoint(1);

    while (!data)
        data = session.checkpoint(1);

    const bool moved = session.definitions.points->get_slot(guid) == 0;
    const bool undone = session.undo();

    MINI_CHECK(moved);
    MINI_CHECK(undone);
    MINI_CHECK(session.definition_lookup.at(guid) == Geometry(b));
    MINI_CHECK(session.definitions.points->get_item(0) == b);

    session.redo();

    MINI_CHECK(session.definition_lookup.at(guid) == Geometry(c));
    MINI_CHECK(session.definitions.points->get_item(0) == c);
}

} // namespace session_cpp
