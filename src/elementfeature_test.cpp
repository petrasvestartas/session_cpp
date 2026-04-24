#include "mini_test.h"
#include "elementfeature.h"
#include "session.h"
#include "point.h"

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("ElementFeature", "Cross Constructor") {
    // uncomment #include "elementfeature.h"
    CrossElementFeature cf;

    MINI_CHECK(cf.face_ids_a.first == -1);
    MINI_CHECK(cf.face_ids_a.second == -1);
    MINI_CHECK(cf.face_ids_b.first == -1);
    MINI_CHECK(cf.face_ids_b.second == -1);
    MINI_CHECK(cf.joint_area_guid.empty());
    MINI_CHECK(cf.joint_volume_guids[0].empty());
    MINI_CHECK(cf.joint_line_guids[0].empty());
    MINI_CHECK(!cf.guid().empty());
}

MINI_TEST("ElementFeature", "Cross Equality") {
    // uncomment #include "elementfeature.h"
    CrossElementFeature a;
    a.face_ids_a = {2, 4};
    a.face_ids_b = {3, 5};
    a.joint_area_guid = "area";
    a.joint_volume_guids = {"v0", "v1"};
    a.joint_line_guids   = {"l0", "l1"};

    CrossElementFeature b = a;
    MINI_CHECK(a == b);
    b.face_ids_a.first = 9;
    MINI_CHECK(a != b);
}

MINI_TEST("ElementFeature", "Cross Json Round Trip") {
    // uncomment #include "elementfeature.h"
    CrossElementFeature a;
    a.face_ids_a = {2, 4};
    a.face_ids_b = {3, 5};
    a.joint_area_guid = "area";
    a.joint_volume_guids = {"v0", "v1"};
    a.joint_line_guids   = {"l0", "l1"};
    (void)a.guid();

    auto json = a.file_json_dumps();
    auto b = CrossElementFeature::file_json_loads(json);
    MINI_CHECK(a == b);
    MINI_CHECK(b.guid() == a.guid());
}

MINI_TEST("ElementFeature", "Cross Pb Round Trip") {
    // uncomment #include "elementfeature.h"
    CrossElementFeature a;
    a.face_ids_a = {2, 4};
    a.face_ids_b = {3, 5};
    a.joint_area_guid = "area";
    a.joint_volume_guids = {"v0", "v1"};
    a.joint_line_guids   = {"l0", "l1"};
    (void)a.guid();

    auto bytes = a.pb_dumps();
    auto b = CrossElementFeature::pb_loads(bytes);
    MINI_CHECK(a == b);
    MINI_CHECK(b.guid() == a.guid());
}

MINI_TEST("ElementFeature", "Face Round Trip") {
    // uncomment #include "elementfeature.h"
    FaceElementFeature a;
    a.face_id_a = 3;
    a.face_id_b = 5;
    a.joint_area_guid = "area";
    (void)a.guid();

    auto bytes = a.pb_dumps();
    auto b = FaceElementFeature::pb_loads(bytes);
    MINI_CHECK(a == b);
    MINI_CHECK(b.guid() == a.guid());

    auto json = a.file_json_dumps();
    auto c = FaceElementFeature::file_json_loads(json);
    MINI_CHECK(a == c);
}

MINI_TEST("ElementFeature", "Axis Round Trip") {
    // uncomment #include "elementfeature.h"
    AxisElementFeature a;
    a.axis_a_guid = "axis_a";
    a.axis_b_guid = "axis_b";
    a.connector_guid = "conn";
    a.distance = 1.5;
    (void)a.guid();

    auto bytes = a.pb_dumps();
    auto b = AxisElementFeature::pb_loads(bytes);
    MINI_CHECK(a == b);
    MINI_CHECK(b.distance == 1.5);
    MINI_CHECK(b.guid() == a.guid());

    auto json = a.file_json_dumps();
    auto c = AxisElementFeature::file_json_loads(json);
    MINI_CHECK(a == c);
}

MINI_TEST("ElementFeature", "Collision Round Trip") {
    // uncomment #include "elementfeature.h"
    CollisionElementFeature a;
    a.mode = "bvh";
    (void)a.guid();

    auto bytes = a.pb_dumps();
    auto b = CollisionElementFeature::pb_loads(bytes);
    MINI_CHECK(a == b);
    MINI_CHECK(b.mode == "bvh");

    auto json = a.file_json_dumps();
    auto c = CollisionElementFeature::file_json_loads(json);
    MINI_CHECK(a == c);
}

MINI_TEST("ElementFeature", "Edge ElementFeature Variant") {
    // uncomment #include "elementfeature.h"
    CrossElementFeature cf;
    cf.joint_area_guid = "area";
    EdgeElementFeature f1 = cf;

    auto bytes = edge_elementfeature_pb_dumps(f1);
    auto f2 = edge_elementfeature_pb_loads(bytes);
    MINI_CHECK(std::holds_alternative<CrossElementFeature>(f2));
    MINI_CHECK(std::get<CrossElementFeature>(f2).joint_area_guid == "area");

    FaceElementFeature ff;
    ff.face_id_a = 2;
    EdgeElementFeature f3 = ff;
    auto f4 = edge_elementfeature_pb_loads(edge_elementfeature_pb_dumps(f3));
    MINI_CHECK(std::holds_alternative<FaceElementFeature>(f4));

    EdgeElementFeature empty{};
    auto f5 = edge_elementfeature_pb_loads(edge_elementfeature_pb_dumps(empty));
    MINI_CHECK(std::holds_alternative<std::monostate>(f5));
}

MINI_TEST("ElementFeature", "Session Add ElementFeature") {
    // uncomment #include "elementfeature.h"
    // uncomment #include "session.h"
    // uncomment #include "point.h"
    Session session;
    auto p1 = std::make_shared<Point>(0, 0, 0);
    auto p2 = std::make_shared<Point>(1, 1, 1);
    session.add_point(p1);
    session.add_point(p2);

    CrossElementFeature cf;
    cf.face_ids_a = {2, 3};
    cf.face_ids_b = {4, 5};
    cf.joint_area_guid = "area_abc";

    auto feat_guid = session.add_elementfeature(p1->guid(), p2->guid(), cf);

    MINI_CHECK(session.edge_elementfeatures.size() == 1);
    MINI_CHECK(session.edge_elementfeatures.count(feat_guid) == 1);
    MINI_CHECK(std::holds_alternative<CrossElementFeature>(session.edge_elementfeatures.at(feat_guid)));
    auto edge_key = std::make_tuple(p1->guid(), p2->guid());
    MINI_CHECK(session.graph.has_edge(edge_key));
}

MINI_TEST("ElementFeature", "Session Pb Round Trip") {
    // uncomment #include "elementfeature.h"
    // uncomment #include "session.h"
    // uncomment #include "point.h"
    Session session;
    auto p1 = std::make_shared<Point>(0, 0, 0);
    auto p2 = std::make_shared<Point>(1, 1, 1);
    session.add_point(p1);
    session.add_point(p2);

    CrossElementFeature cf;
    cf.face_ids_a = {2, 3};
    cf.face_ids_b = {4, 5};
    cf.joint_area_guid = "area_abc";
    cf.joint_volume_guids = {"v0", "v1"};
    auto feat_guid = session.add_elementfeature(p1->guid(), p2->guid(), cf);

    auto bytes = session.pb_dumps();
    auto session2 = Session::pb_loads(bytes);

    MINI_CHECK(session2.edge_elementfeatures.size() == 1);
    MINI_CHECK(session2.edge_elementfeatures.count(feat_guid) == 1);
    auto& recovered = std::get<CrossElementFeature>(session2.edge_elementfeatures.at(feat_guid));
    MINI_CHECK(recovered.face_ids_a.first == 2);
    MINI_CHECK(recovered.face_ids_b.second == 5);
    MINI_CHECK(recovered.joint_area_guid == "area_abc");
    MINI_CHECK(recovered.joint_volume_guids[0] == "v0");
    MINI_CHECK(recovered.joint_volume_guids[1] == "v1");
}

} // namespace session_cpp
