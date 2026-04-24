#include "elementfeature.h"
#include "elementfeature.pb.h"
#include "fmt/core.h"
#include <sstream>

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////
// CrossElementFeature
///////////////////////////////////////////////////////////////////////////////

bool CrossElementFeature::operator==(const CrossElementFeature& o) const {
    return face_ids_a == o.face_ids_a
        && face_ids_b == o.face_ids_b
        && joint_area_guid == o.joint_area_guid
        && joint_volume_guids == o.joint_volume_guids
        && joint_line_guids == o.joint_line_guids;
}

std::string CrossElementFeature::str() const {
    return fmt::format("CrossElementFeature(area={}, vol=[{},{}], lines=[{},{}])",
        joint_area_guid.substr(0, 8),
        joint_volume_guids[0].substr(0, 8), joint_volume_guids[1].substr(0, 8),
        joint_line_guids[0].substr(0, 8), joint_line_guids[1].substr(0, 8));
}

std::string CrossElementFeature::repr() const {
    std::ostringstream os;
    os << "CrossElementFeature(face_ids_a=(" << face_ids_a.first << "," << face_ids_a.second << ")"
       << ", face_ids_b=(" << face_ids_b.first << "," << face_ids_b.second << ")"
       << ", joint_area_guid=" << joint_area_guid
       << ", joint_volume_guids=[" << joint_volume_guids[0] << "," << joint_volume_guids[1] << "]"
       << ", joint_line_guids=[" << joint_line_guids[0] << "," << joint_line_guids[1] << "])";
    return os.str();
}

nlohmann::ordered_json CrossElementFeature::jsondump() const {
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

std::string CrossElementFeature::file_json_dumps() const { return jsondump().dump(); }

CrossElementFeature CrossElementFeature::jsonload(const nlohmann::ordered_json& data) {
    CrossElementFeature f;
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

CrossElementFeature CrossElementFeature::file_json_loads(const std::string& s) {
    return jsonload(nlohmann::ordered_json::parse(s));
}

std::string CrossElementFeature::pb_dumps() const {
    session_proto::CrossElementFeature proto;
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

CrossElementFeature CrossElementFeature::pb_loads(const std::string& data) {
    session_proto::CrossElementFeature proto;
    proto.ParseFromString(data);
    CrossElementFeature f;
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
// FaceElementFeature
///////////////////////////////////////////////////////////////////////////////

bool FaceElementFeature::operator==(const FaceElementFeature& o) const {
    return face_id_a == o.face_id_a
        && face_id_b == o.face_id_b
        && joint_area_guid == o.joint_area_guid;
}

std::string FaceElementFeature::str() const {
    return fmt::format("FaceElementFeature(face_a={}, face_b={}, area={})",
        face_id_a, face_id_b, joint_area_guid.substr(0, 8));
}

std::string FaceElementFeature::repr() const {
    std::ostringstream os;
    os << "FaceElementFeature(face_id_a=" << face_id_a
       << ", face_id_b=" << face_id_b
       << ", joint_area_guid=" << joint_area_guid << ")";
    return os.str();
}

nlohmann::ordered_json FaceElementFeature::jsondump() const {
    nlohmann::ordered_json data;
    data["face_id_a"] = face_id_a;
    data["face_id_b"] = face_id_b;
    data["guid"] = guid();
    data["joint_area_guid"] = joint_area_guid;
    return data;
}

std::string FaceElementFeature::file_json_dumps() const { return jsondump().dump(); }

FaceElementFeature FaceElementFeature::jsonload(const nlohmann::ordered_json& data) {
    FaceElementFeature f;
    f.face_id_a = data.value("face_id_a", -1);
    f.face_id_b = data.value("face_id_b", -1);
    if (data.contains("joint_area_guid")) f.joint_area_guid = data["joint_area_guid"].get<std::string>();
    if (data.contains("guid")) f._guid = data["guid"].get<std::string>();
    return f;
}

FaceElementFeature FaceElementFeature::file_json_loads(const std::string& s) {
    return jsonload(nlohmann::ordered_json::parse(s));
}

std::string FaceElementFeature::pb_dumps() const {
    session_proto::FaceElementFeature proto;
    proto.set_guid(guid());
    proto.set_face_id_a(face_id_a);
    proto.set_face_id_b(face_id_b);
    proto.set_joint_area_guid(joint_area_guid);
    return proto.SerializeAsString();
}

FaceElementFeature FaceElementFeature::pb_loads(const std::string& data) {
    session_proto::FaceElementFeature proto;
    proto.ParseFromString(data);
    FaceElementFeature f;
    f._guid = proto.guid();
    f.face_id_a = proto.face_id_a();
    f.face_id_b = proto.face_id_b();
    f.joint_area_guid = proto.joint_area_guid();
    return f;
}

///////////////////////////////////////////////////////////////////////////////
// AxisElementFeature
///////////////////////////////////////////////////////////////////////////////

bool AxisElementFeature::operator==(const AxisElementFeature& o) const {
    return axis_a_guid == o.axis_a_guid
        && axis_b_guid == o.axis_b_guid
        && connector_guid == o.connector_guid
        && distance == o.distance;
}

std::string AxisElementFeature::str() const {
    return fmt::format("AxisElementFeature(a={}, b={}, dist={:.3f})",
        axis_a_guid.substr(0, 8), axis_b_guid.substr(0, 8), distance);
}

std::string AxisElementFeature::repr() const {
    std::ostringstream os;
    os << "AxisElementFeature(axis_a_guid=" << axis_a_guid
       << ", axis_b_guid=" << axis_b_guid
       << ", connector_guid=" << connector_guid
       << ", distance=" << distance << ")";
    return os.str();
}

nlohmann::ordered_json AxisElementFeature::jsondump() const {
    nlohmann::ordered_json data;
    data["axis_a_guid"] = axis_a_guid;
    data["axis_b_guid"] = axis_b_guid;
    data["connector_guid"] = connector_guid;
    data["distance"] = distance;
    data["guid"] = guid();
    return data;
}

std::string AxisElementFeature::file_json_dumps() const { return jsondump().dump(); }

AxisElementFeature AxisElementFeature::jsonload(const nlohmann::ordered_json& data) {
    AxisElementFeature f;
    if (data.contains("axis_a_guid"))    f.axis_a_guid = data["axis_a_guid"].get<std::string>();
    if (data.contains("axis_b_guid"))    f.axis_b_guid = data["axis_b_guid"].get<std::string>();
    if (data.contains("connector_guid")) f.connector_guid = data["connector_guid"].get<std::string>();
    f.distance = data.value("distance", 0.0);
    if (data.contains("guid")) f._guid = data["guid"].get<std::string>();
    return f;
}

AxisElementFeature AxisElementFeature::file_json_loads(const std::string& s) {
    return jsonload(nlohmann::ordered_json::parse(s));
}

std::string AxisElementFeature::pb_dumps() const {
    session_proto::AxisElementFeature proto;
    proto.set_guid(guid());
    proto.set_axis_a_guid(axis_a_guid);
    proto.set_axis_b_guid(axis_b_guid);
    proto.set_connector_guid(connector_guid);
    proto.set_distance(distance);
    return proto.SerializeAsString();
}

AxisElementFeature AxisElementFeature::pb_loads(const std::string& data) {
    session_proto::AxisElementFeature proto;
    proto.ParseFromString(data);
    AxisElementFeature f;
    f._guid = proto.guid();
    f.axis_a_guid = proto.axis_a_guid();
    f.axis_b_guid = proto.axis_b_guid();
    f.connector_guid = proto.connector_guid();
    f.distance = proto.distance();
    return f;
}

///////////////////////////////////////////////////////////////////////////////
// CollisionElementFeature
///////////////////////////////////////////////////////////////////////////////

bool CollisionElementFeature::operator==(const CollisionElementFeature& o) const {
    return mode == o.mode;
}

std::string CollisionElementFeature::str() const {
    return fmt::format("CollisionElementFeature(mode={})", mode);
}

std::string CollisionElementFeature::repr() const {
    return "CollisionElementFeature(mode=\"" + mode + "\")";
}

nlohmann::ordered_json CollisionElementFeature::jsondump() const {
    nlohmann::ordered_json data;
    data["guid"] = guid();
    data["mode"] = mode;
    return data;
}

std::string CollisionElementFeature::file_json_dumps() const { return jsondump().dump(); }

CollisionElementFeature CollisionElementFeature::jsonload(const nlohmann::ordered_json& data) {
    CollisionElementFeature f;
    f.mode = data.value("mode", std::string("bvh"));
    if (data.contains("guid")) f._guid = data["guid"].get<std::string>();
    return f;
}

CollisionElementFeature CollisionElementFeature::file_json_loads(const std::string& s) {
    return jsonload(nlohmann::ordered_json::parse(s));
}

std::string CollisionElementFeature::pb_dumps() const {
    session_proto::CollisionElementFeature proto;
    proto.set_guid(guid());
    proto.set_mode(mode);
    return proto.SerializeAsString();
}

CollisionElementFeature CollisionElementFeature::pb_loads(const std::string& data) {
    session_proto::CollisionElementFeature proto;
    proto.ParseFromString(data);
    CollisionElementFeature f;
    f._guid = proto.guid();
    f.mode = proto.mode();
    return f;
}

///////////////////////////////////////////////////////////////////////////////
// EdgeElementFeature variant dispatch
///////////////////////////////////////////////////////////////////////////////

std::string edge_elementfeature_pb_dumps(const EdgeElementFeature& f) {
    session_proto::EdgeElementFeature proto;
    if (auto* cf = std::get_if<CrossElementFeature>(&f)) {
        proto.mutable_cross()->ParseFromString(cf->pb_dumps());
    } else if (auto* ff = std::get_if<FaceElementFeature>(&f)) {
        proto.mutable_face()->ParseFromString(ff->pb_dumps());
    } else if (auto* af = std::get_if<AxisElementFeature>(&f)) {
        proto.mutable_axis()->ParseFromString(af->pb_dumps());
    } else if (auto* col = std::get_if<CollisionElementFeature>(&f)) {
        proto.mutable_collision()->ParseFromString(col->pb_dumps());
    }
    return proto.SerializeAsString();
}

EdgeElementFeature edge_elementfeature_pb_loads(const std::string& data) {
    session_proto::EdgeElementFeature proto;
    proto.ParseFromString(data);
    switch (proto.payload_case()) {
        case session_proto::EdgeElementFeature::kCross:
            return CrossElementFeature::pb_loads(proto.cross().SerializeAsString());
        case session_proto::EdgeElementFeature::kFace:
            return FaceElementFeature::pb_loads(proto.face().SerializeAsString());
        case session_proto::EdgeElementFeature::kAxis:
            return AxisElementFeature::pb_loads(proto.axis().SerializeAsString());
        case session_proto::EdgeElementFeature::kCollision:
            return CollisionElementFeature::pb_loads(proto.collision().SerializeAsString());
        default:
            return std::monostate{};
    }
}

nlohmann::ordered_json edge_elementfeature_jsondump(const EdgeElementFeature& f) {
    nlohmann::ordered_json data;
    if (auto* cf = std::get_if<CrossElementFeature>(&f)) {
        data["kind"] = "CrossElementFeature";
        data["data"] = cf->jsondump();
    } else if (auto* ff = std::get_if<FaceElementFeature>(&f)) {
        data["kind"] = "FaceElementFeature";
        data["data"] = ff->jsondump();
    } else if (auto* af = std::get_if<AxisElementFeature>(&f)) {
        data["kind"] = "AxisElementFeature";
        data["data"] = af->jsondump();
    } else if (auto* col = std::get_if<CollisionElementFeature>(&f)) {
        data["kind"] = "CollisionElementFeature";
        data["data"] = col->jsondump();
    } else {
        data["kind"] = "Empty";
    }
    return data;
}

EdgeElementFeature edge_elementfeature_jsonload(const nlohmann::ordered_json& data) {
    std::string kind = data.value("kind", "Empty");
    if (kind == "CrossElementFeature")     return CrossElementFeature::jsonload(data["data"]);
    if (kind == "FaceElementFeature")      return FaceElementFeature::jsonload(data["data"]);
    if (kind == "AxisElementFeature")      return AxisElementFeature::jsonload(data["data"]);
    if (kind == "CollisionElementFeature") return CollisionElementFeature::jsonload(data["data"]);
    return std::monostate{};
}

std::string edge_elementfeature_guid(const EdgeElementFeature& f) {
    if (auto* cf = std::get_if<CrossElementFeature>(&f))      return cf->guid();
    if (auto* ff = std::get_if<FaceElementFeature>(&f))       return ff->guid();
    if (auto* af = std::get_if<AxisElementFeature>(&f))       return af->guid();
    if (auto* col = std::get_if<CollisionElementFeature>(&f)) return col->guid();
    return "";
}

} // namespace session_cpp
