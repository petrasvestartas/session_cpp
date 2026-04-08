#include "mini_test.h"
#include "feature.h"
#include "session.h"
#include "point.h"

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Feature", "Cross Constructor") {
    // uncomment #include "feature.h"
    CrossFeature cf;

    MINI_CHECK(cf.face_ids_a.first == -1);
    MINI_CHECK(cf.face_ids_a.second == -1);
    MINI_CHECK(cf.face_ids_b.first == -1);
    MINI_CHECK(cf.face_ids_b.second == -1);
    MINI_CHECK(cf.joint_area_guid.empty());
    MINI_CHECK(cf.joint_volume_guids[0].empty());
    MINI_CHECK(cf.joint_line_guids[0].empty());
    MINI_CHECK(!cf.guid().empty());
}

MINI_TEST("Feature", "Cross Equality") {
    // uncomment #include "feature.h"
    CrossFeature a;
    a.face_ids_a = {2, 4};
    a.face_ids_b = {3, 5};
    a.joint_area_guid = "area";
    a.joint_volume_guids = {"v0", "v1"};
    a.joint_line_guids   = {"l0", "l1"};

    CrossFeature b = a;
    MINI_CHECK(a == b);
    b.face_ids_a.first = 9;
    MINI_CHECK(a != b);
}

MINI_TEST("Feature", "Cross Json Round Trip") {
    // uncomment #include "feature.h"
    CrossFeature a;
    a.face_ids_a = {2, 4};
    a.face_ids_b = {3, 5};
    a.joint_area_guid = "area";
    a.joint_volume_guids = {"v0", "v1"};
    a.joint_line_guids   = {"l0", "l1"};
    (void)a.guid();

    auto json = a.json_dumps();
    auto b = CrossFeature::json_loads(json);
    MINI_CHECK(a == b);
    MINI_CHECK(b.guid() == a.guid());
}

MINI_TEST("Feature", "Cross Pb Round Trip") {
    // uncomment #include "feature.h"
    CrossFeature a;
    a.face_ids_a = {2, 4};
    a.face_ids_b = {3, 5};
    a.joint_area_guid = "area";
    a.joint_volume_guids = {"v0", "v1"};
    a.joint_line_guids   = {"l0", "l1"};
    (void)a.guid();

    auto bytes = a.pb_dumps();
    auto b = CrossFeature::pb_loads(bytes);
    MINI_CHECK(a == b);
    MINI_CHECK(b.guid() == a.guid());
}

MINI_TEST("Feature", "Face Round Trip") {
    // uncomment #include "feature.h"
    FaceFeature a;
    a.face_id_a = 3;
    a.face_id_b = 5;
    a.joint_area_guid = "area";
    (void)a.guid();

    auto bytes = a.pb_dumps();
    auto b = FaceFeature::pb_loads(bytes);
    MINI_CHECK(a == b);
    MINI_CHECK(b.guid() == a.guid());

    auto json = a.json_dumps();
    auto c = FaceFeature::json_loads(json);
    MINI_CHECK(a == c);
}

MINI_TEST("Feature", "Axis Round Trip") {
    // uncomment #include "feature.h"
    AxisFeature a;
    a.axis_a_guid = "axis_a";
    a.axis_b_guid = "axis_b";
    a.connector_guid = "conn";
    a.distance = 1.5;
    (void)a.guid();

    auto bytes = a.pb_dumps();
    auto b = AxisFeature::pb_loads(bytes);
    MINI_CHECK(a == b);
    MINI_CHECK(b.distance == 1.5);
    MINI_CHECK(b.guid() == a.guid());

    auto json = a.json_dumps();
    auto c = AxisFeature::json_loads(json);
    MINI_CHECK(a == c);
}

MINI_TEST("Feature", "Collision Round Trip") {
    // uncomment #include "feature.h"
    CollisionFeature a;
    a.mode = "bvh";
    (void)a.guid();

    auto bytes = a.pb_dumps();
    auto b = CollisionFeature::pb_loads(bytes);
    MINI_CHECK(a == b);
    MINI_CHECK(b.mode == "bvh");

    auto json = a.json_dumps();
    auto c = CollisionFeature::json_loads(json);
    MINI_CHECK(a == c);
}

MINI_TEST("Feature", "Edge Feature Variant") {
    // uncomment #include "feature.h"
    CrossFeature cf;
    cf.joint_area_guid = "area";
    EdgeFeature f1 = cf;

    auto bytes = edge_feature_pb_dumps(f1);
    auto f2 = edge_feature_pb_loads(bytes);
    MINI_CHECK(std::holds_alternative<CrossFeature>(f2));
    MINI_CHECK(std::get<CrossFeature>(f2).joint_area_guid == "area");

    FaceFeature ff;
    ff.face_id_a = 2;
    EdgeFeature f3 = ff;
    auto f4 = edge_feature_pb_loads(edge_feature_pb_dumps(f3));
    MINI_CHECK(std::holds_alternative<FaceFeature>(f4));

    EdgeFeature empty{};
    auto f5 = edge_feature_pb_loads(edge_feature_pb_dumps(empty));
    MINI_CHECK(std::holds_alternative<std::monostate>(f5));
}

MINI_TEST("Feature", "Session Add Feature") {
    // uncomment #include "feature.h"
    // uncomment #include "session.h"
    // uncomment #include "point.h"
    Session session;
    auto p1 = std::make_shared<Point>(0, 0, 0);
    auto p2 = std::make_shared<Point>(1, 1, 1);
    session.add_point(p1);
    session.add_point(p2);

    CrossFeature cf;
    cf.face_ids_a = {2, 3};
    cf.face_ids_b = {4, 5};
    cf.joint_area_guid = "area_abc";

    auto feat_guid = session.add_feature(p1->guid(), p2->guid(), cf);

    MINI_CHECK(session.edge_features.size() == 1);
    MINI_CHECK(session.edge_features.count(feat_guid) == 1);
    MINI_CHECK(std::holds_alternative<CrossFeature>(session.edge_features.at(feat_guid)));
    auto edge_key = std::make_tuple(p1->guid(), p2->guid());
    MINI_CHECK(session.graph.has_edge(edge_key));
}

MINI_TEST("Feature", "Session Pb Round Trip") {
    // uncomment #include "feature.h"
    // uncomment #include "session.h"
    // uncomment #include "point.h"
    Session session;
    auto p1 = std::make_shared<Point>(0, 0, 0);
    auto p2 = std::make_shared<Point>(1, 1, 1);
    session.add_point(p1);
    session.add_point(p2);

    CrossFeature cf;
    cf.face_ids_a = {2, 3};
    cf.face_ids_b = {4, 5};
    cf.joint_area_guid = "area_abc";
    cf.joint_volume_guids = {"v0", "v1"};
    auto feat_guid = session.add_feature(p1->guid(), p2->guid(), cf);

    auto bytes = session.pb_dumps();
    auto session2 = Session::pb_loads(bytes);

    MINI_CHECK(session2.edge_features.size() == 1);
    MINI_CHECK(session2.edge_features.count(feat_guid) == 1);
    auto& recovered = std::get<CrossFeature>(session2.edge_features.at(feat_guid));
    MINI_CHECK(recovered.face_ids_a.first == 2);
    MINI_CHECK(recovered.face_ids_b.second == 5);
    MINI_CHECK(recovered.joint_area_guid == "area_abc");
    MINI_CHECK(recovered.joint_volume_guids[0] == "v0");
    MINI_CHECK(recovered.joint_volume_guids[1] == "v1");
}

} // namespace session_cpp
