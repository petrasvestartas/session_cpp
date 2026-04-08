#include "feature.h"
#include "feature.pb.h"
#include "fmt/core.h"
#include <sstream>

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////
// CrossFeature
///////////////////////////////////////////////////////////////////////////////

bool CrossFeature::operator==(const CrossFeature& o) const {
    return face_ids_a == o.face_ids_a
        && face_ids_b == o.face_ids_b
        && joint_area_guid == o.joint_area_guid
        && joint_volume_guids == o.joint_volume_guids
        && joint_line_guids == o.joint_line_guids;
}

std::string CrossFeature::str() const {
    return fmt::format("CrossFeature(area={}, vol=[{},{}], lines=[{},{}])",
        joint_area_guid.substr(0, 8),
        joint_volume_guids[0].substr(0, 8), joint_volume_guids[1].substr(0, 8),
        joint_line_guids[0].substr(0, 8), joint_line_guids[1].substr(0, 8));
}

std::string CrossFeature::repr() const {
    std::ostringstream os;
    os << "CrossFeature(face_ids_a=(" << face_ids_a.first << "," << face_ids_a.second << ")"
       << ", face_ids_b=(" << face_ids_b.first << "," << face_ids_b.second << ")"
       << ", joint_area_guid=" << joint_area_guid
       << ", joint_volume_guids=[" << joint_volume_guids[0] << "," << joint_volume_guids[1] << "]"
       << ", joint_line_guids=[" << joint_line_guids[0] << "," << joint_line_guids[1] << "])";
    return os.str();
}

nlohmann::ordered_json CrossFeature::jsondump() const {
    nlohmann::ordered_json data;
    data["face_ids_a_first"]  = face_ids_a.first;
    data["face_ids_a_second"] = face_ids_a.second;
    data["face_ids_b_first"]  = face_ids_b.first;
    data["face_ids_b_second"] = face_ids_b.second;
    data["guid"] = guid();
    data["joint_area_guid"]    = joint_area_guid;
    data["joint_line_guids"]   = std::vector<std::string>{joint_line_guids[0], joint_line_guids[1]};
    data["joint_volume_guids"] = std::vector<std::string>{joint_volume_guids[0], joint_volume_guids[1]};
    return data;
}

std::string CrossFeature::json_dumps() const { return jsondump().dump(); }

CrossFeature CrossFeature::jsonload(const nlohmann::ordered_json& data) {
    CrossFeature f;
    f.face_ids_a.first  = data.value("face_ids_a_first",  -1);
    f.face_ids_a.second = data.value("face_ids_a_second", -1);
    f.face_ids_b.first  = data.value("face_ids_b_first",  -1);
    f.face_ids_b.second = data.value("face_ids_b_second", -1);
    if (data.contains("joint_area_guid")) f.joint_area_guid = data["joint_area_guid"].get<std::string>();
    if (data.contains("joint_line_guids")) {
        auto arr = data["joint_line_guids"].get<std::vector<std::string>>();
        if (arr.size() > 0) f.joint_line_guids[0] = arr[0];
        if (arr.size() > 1) f.joint_line_guids[1] = arr[1];
    }
    if (data.contains("joint_volume_guids")) {
        auto arr = data["joint_volume_guids"].get<std::vector<std::string>>();
        if (arr.size() > 0) f.joint_volume_guids[0] = arr[0];
        if (arr.size() > 1) f.joint_volume_guids[1] = arr[1];
    }
    if (data.contains("guid")) f._guid = data["guid"].get<std::string>();
    return f;
}

CrossFeature CrossFeature::json_loads(const std::string& s) {
    return jsonload(nlohmann::ordered_json::parse(s));
}

std::string CrossFeature::pb_dumps() const {
    session_proto::CrossFeature proto;
    proto.set_guid(guid());
    proto.set_face_ids_a_first(face_ids_a.first);
    proto.set_face_ids_a_second(face_ids_a.second);
    proto.set_face_ids_b_first(face_ids_b.first);
    proto.set_face_ids_b_second(face_ids_b.second);
    proto.set_joint_area_guid(joint_area_guid);
    for (const auto& g : joint_volume_guids) proto.add_joint_volume_guids(g);
    for (const auto& g : joint_line_guids)   proto.add_joint_line_guids(g);
    return proto.SerializeAsString();
}

CrossFeature CrossFeature::pb_loads(const std::string& data) {
    session_proto::CrossFeature proto;
    proto.ParseFromString(data);
    CrossFeature f;
    f._guid = proto.guid();
    f.face_ids_a.first  = proto.face_ids_a_first();
    f.face_ids_a.second = proto.face_ids_a_second();
    f.face_ids_b.first  = proto.face_ids_b_first();
    f.face_ids_b.second = proto.face_ids_b_second();
    f.joint_area_guid = proto.joint_area_guid();
    if (proto.joint_volume_guids_size() > 0) f.joint_volume_guids[0] = proto.joint_volume_guids(0);
    if (proto.joint_volume_guids_size() > 1) f.joint_volume_guids[1] = proto.joint_volume_guids(1);
    if (proto.joint_line_guids_size() > 0)   f.joint_line_guids[0]   = proto.joint_line_guids(0);
    if (proto.joint_line_guids_size() > 1)   f.joint_line_guids[1]   = proto.joint_line_guids(1);
    return f;
}

///////////////////////////////////////////////////////////////////////////////
// FaceFeature
///////////////////////////////////////////////////////////////////////////////

bool FaceFeature::operator==(const FaceFeature& o) const {
    return face_id_a == o.face_id_a
        && face_id_b == o.face_id_b
        && joint_area_guid == o.joint_area_guid;
}

std::string FaceFeature::str() const {
    return fmt::format("FaceFeature(face_a={}, face_b={}, area={})",
        face_id_a, face_id_b, joint_area_guid.substr(0, 8));
}

std::string FaceFeature::repr() const {
    std::ostringstream os;
    os << "FaceFeature(face_id_a=" << face_id_a
       << ", face_id_b=" << face_id_b
       << ", joint_area_guid=" << joint_area_guid << ")";
    return os.str();
}

nlohmann::ordered_json FaceFeature::jsondump() const {
    nlohmann::ordered_json data;
    data["face_id_a"] = face_id_a;
    data["face_id_b"] = face_id_b;
    data["guid"] = guid();
    data["joint_area_guid"] = joint_area_guid;
    return data;
}

std::string FaceFeature::json_dumps() const { return jsondump().dump(); }

FaceFeature FaceFeature::jsonload(const nlohmann::ordered_json& data) {
    FaceFeature f;
    f.face_id_a = data.value("face_id_a", -1);
    f.face_id_b = data.value("face_id_b", -1);
    if (data.contains("joint_area_guid")) f.joint_area_guid = data["joint_area_guid"].get<std::string>();
    if (data.contains("guid")) f._guid = data["guid"].get<std::string>();
    return f;
}

FaceFeature FaceFeature::json_loads(const std::string& s) {
    return jsonload(nlohmann::ordered_json::parse(s));
}

std::string FaceFeature::pb_dumps() const {
    session_proto::FaceFeature proto;
    proto.set_guid(guid());
    proto.set_face_id_a(face_id_a);
    proto.set_face_id_b(face_id_b);
    proto.set_joint_area_guid(joint_area_guid);
    return proto.SerializeAsString();
}

FaceFeature FaceFeature::pb_loads(const std::string& data) {
    session_proto::FaceFeature proto;
    proto.ParseFromString(data);
    FaceFeature f;
    f._guid = proto.guid();
    f.face_id_a = proto.face_id_a();
    f.face_id_b = proto.face_id_b();
    f.joint_area_guid = proto.joint_area_guid();
    return f;
}

///////////////////////////////////////////////////////////////////////////////
// AxisFeature
///////////////////////////////////////////////////////////////////////////////

bool AxisFeature::operator==(const AxisFeature& o) const {
    return axis_a_guid == o.axis_a_guid
        && axis_b_guid == o.axis_b_guid
        && connector_guid == o.connector_guid
        && distance == o.distance;
}

std::string AxisFeature::str() const {
    return fmt::format("AxisFeature(a={}, b={}, dist={:.3f})",
        axis_a_guid.substr(0, 8), axis_b_guid.substr(0, 8), distance);
}

std::string AxisFeature::repr() const {
    std::ostringstream os;
    os << "AxisFeature(axis_a_guid=" << axis_a_guid
       << ", axis_b_guid=" << axis_b_guid
       << ", connector_guid=" << connector_guid
       << ", distance=" << distance << ")";
    return os.str();
}

nlohmann::ordered_json AxisFeature::jsondump() const {
    nlohmann::ordered_json data;
    data["axis_a_guid"] = axis_a_guid;
    data["axis_b_guid"] = axis_b_guid;
    data["connector_guid"] = connector_guid;
    data["distance"] = distance;
    data["guid"] = guid();
    return data;
}

std::string AxisFeature::json_dumps() const { return jsondump().dump(); }

AxisFeature AxisFeature::jsonload(const nlohmann::ordered_json& data) {
    AxisFeature f;
    if (data.contains("axis_a_guid"))    f.axis_a_guid = data["axis_a_guid"].get<std::string>();
    if (data.contains("axis_b_guid"))    f.axis_b_guid = data["axis_b_guid"].get<std::string>();
    if (data.contains("connector_guid")) f.connector_guid = data["connector_guid"].get<std::string>();
    f.distance = data.value("distance", 0.0);
    if (data.contains("guid")) f._guid = data["guid"].get<std::string>();
    return f;
}

AxisFeature AxisFeature::json_loads(const std::string& s) {
    return jsonload(nlohmann::ordered_json::parse(s));
}

std::string AxisFeature::pb_dumps() const {
    session_proto::AxisFeature proto;
    proto.set_guid(guid());
    proto.set_axis_a_guid(axis_a_guid);
    proto.set_axis_b_guid(axis_b_guid);
    proto.set_connector_guid(connector_guid);
    proto.set_distance(distance);
    return proto.SerializeAsString();
}

AxisFeature AxisFeature::pb_loads(const std::string& data) {
    session_proto::AxisFeature proto;
    proto.ParseFromString(data);
    AxisFeature f;
    f._guid = proto.guid();
    f.axis_a_guid = proto.axis_a_guid();
    f.axis_b_guid = proto.axis_b_guid();
    f.connector_guid = proto.connector_guid();
    f.distance = proto.distance();
    return f;
}

///////////////////////////////////////////////////////////////////////////////
// CollisionFeature
///////////////////////////////////////////////////////////////////////////////

bool CollisionFeature::operator==(const CollisionFeature& o) const {
    return mode == o.mode;
}

std::string CollisionFeature::str() const {
    return fmt::format("CollisionFeature(mode={})", mode);
}

std::string CollisionFeature::repr() const {
    return "CollisionFeature(mode=\"" + mode + "\")";
}

nlohmann::ordered_json CollisionFeature::jsondump() const {
    nlohmann::ordered_json data;
    data["guid"] = guid();
    data["mode"] = mode;
    return data;
}

std::string CollisionFeature::json_dumps() const { return jsondump().dump(); }

CollisionFeature CollisionFeature::jsonload(const nlohmann::ordered_json& data) {
    CollisionFeature f;
    f.mode = data.value("mode", std::string("bvh"));
    if (data.contains("guid")) f._guid = data["guid"].get<std::string>();
    return f;
}

CollisionFeature CollisionFeature::json_loads(const std::string& s) {
    return jsonload(nlohmann::ordered_json::parse(s));
}

std::string CollisionFeature::pb_dumps() const {
    session_proto::CollisionFeature proto;
    proto.set_guid(guid());
    proto.set_mode(mode);
    return proto.SerializeAsString();
}

CollisionFeature CollisionFeature::pb_loads(const std::string& data) {
    session_proto::CollisionFeature proto;
    proto.ParseFromString(data);
    CollisionFeature f;
    f._guid = proto.guid();
    f.mode = proto.mode();
    return f;
}

///////////////////////////////////////////////////////////////////////////////
// EdgeFeature variant dispatch
///////////////////////////////////////////////////////////////////////////////

std::string edge_feature_pb_dumps(const EdgeFeature& f) {
    session_proto::EdgeFeature proto;
    if (auto* cf = std::get_if<CrossFeature>(&f)) {
        proto.mutable_cross()->ParseFromString(cf->pb_dumps());
    } else if (auto* ff = std::get_if<FaceFeature>(&f)) {
        proto.mutable_face()->ParseFromString(ff->pb_dumps());
    } else if (auto* af = std::get_if<AxisFeature>(&f)) {
        proto.mutable_axis()->ParseFromString(af->pb_dumps());
    } else if (auto* col = std::get_if<CollisionFeature>(&f)) {
        proto.mutable_collision()->ParseFromString(col->pb_dumps());
    }
    return proto.SerializeAsString();
}

EdgeFeature edge_feature_pb_loads(const std::string& data) {
    session_proto::EdgeFeature proto;
    proto.ParseFromString(data);
    switch (proto.payload_case()) {
        case session_proto::EdgeFeature::kCross:
            return CrossFeature::pb_loads(proto.cross().SerializeAsString());
        case session_proto::EdgeFeature::kFace:
            return FaceFeature::pb_loads(proto.face().SerializeAsString());
        case session_proto::EdgeFeature::kAxis:
            return AxisFeature::pb_loads(proto.axis().SerializeAsString());
        case session_proto::EdgeFeature::kCollision:
            return CollisionFeature::pb_loads(proto.collision().SerializeAsString());
        default:
            return std::monostate{};
    }
}

nlohmann::ordered_json edge_feature_jsondump(const EdgeFeature& f) {
    nlohmann::ordered_json data;
    if (auto* cf = std::get_if<CrossFeature>(&f)) {
        data["kind"] = "CrossFeature";
        data["data"] = cf->jsondump();
    } else if (auto* ff = std::get_if<FaceFeature>(&f)) {
        data["kind"] = "FaceFeature";
        data["data"] = ff->jsondump();
    } else if (auto* af = std::get_if<AxisFeature>(&f)) {
        data["kind"] = "AxisFeature";
        data["data"] = af->jsondump();
    } else if (auto* col = std::get_if<CollisionFeature>(&f)) {
        data["kind"] = "CollisionFeature";
        data["data"] = col->jsondump();
    } else {
        data["kind"] = "Empty";
    }
    return data;
}

EdgeFeature edge_feature_jsonload(const nlohmann::ordered_json& data) {
    std::string kind = data.value("kind", "Empty");
    if (kind == "CrossFeature")     return CrossFeature::jsonload(data["data"]);
    if (kind == "FaceFeature")      return FaceFeature::jsonload(data["data"]);
    if (kind == "AxisFeature")      return AxisFeature::jsonload(data["data"]);
    if (kind == "CollisionFeature") return CollisionFeature::jsonload(data["data"]);
    return std::monostate{};
}

std::string edge_feature_guid(const EdgeFeature& f) {
    if (auto* cf = std::get_if<CrossFeature>(&f))     return cf->guid();
    if (auto* ff = std::get_if<FaceFeature>(&f))      return ff->guid();
    if (auto* af = std::get_if<AxisFeature>(&f))      return af->guid();
    if (auto* col = std::get_if<CollisionFeature>(&f)) return col->guid();
    return "";
}

} // namespace session_cpp
