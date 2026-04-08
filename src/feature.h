#pragma once
#include "guid.h"
#include "json.h"
#include <array>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace session_cpp {

/**
 * @brief CrossFeature — plane-to-face cross joint between plate elements.
 *
 * Geometry (joint_area, joint_lines, joint_volumes) lives in Session::lookup
 * as Polylines and is referenced here by GUID.
 */
struct CrossFeature {
    std::pair<int,int> face_ids_a{-1,-1};
    std::pair<int,int> face_ids_b{-1,-1};
    std::string joint_area_guid;
    std::array<std::string,2> joint_volume_guids;
    std::array<std::string,2> joint_line_guids;

    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

    bool operator==(const CrossFeature& o) const;
    bool operator!=(const CrossFeature& o) const { return !(*this == o); }
    std::string str() const;
    std::string repr() const;

    nlohmann::ordered_json jsondump() const;
    std::string json_dumps() const;
    static CrossFeature jsonload(const nlohmann::ordered_json& data);
    static CrossFeature json_loads(const std::string& s);

    std::string pb_dumps() const;
    static CrossFeature pb_loads(const std::string& data);

private:
    mutable std::string _guid;
};

/**
 * @brief FaceFeature — face-to-face coplanar joint between two elements.
 */
struct FaceFeature {
    int face_id_a = -1;
    int face_id_b = -1;
    std::string joint_area_guid;

    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

    bool operator==(const FaceFeature& o) const;
    bool operator!=(const FaceFeature& o) const { return !(*this == o); }
    std::string str() const;
    std::string repr() const;

    nlohmann::ordered_json jsondump() const;
    std::string json_dumps() const;
    static FaceFeature jsonload(const nlohmann::ordered_json& data);
    static FaceFeature json_loads(const std::string& s);

    std::string pb_dumps() const;
    static FaceFeature pb_loads(const std::string& data);

private:
    mutable std::string _guid;
};

/**
 * @brief AxisFeature — closest-polyline / axis-to-axis connection between
 * two axis-based elements (e.g. beams). Stores the GUIDs of the two axes,
 * the connector segment between them, and the minimum distance.
 */
struct AxisFeature {
    std::string axis_a_guid;
    std::string axis_b_guid;
    std::string connector_guid;
    double distance = 0.0;

    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

    bool operator==(const AxisFeature& o) const;
    bool operator!=(const AxisFeature& o) const { return !(*this == o); }
    std::string str() const;
    std::string repr() const;

    nlohmann::ordered_json jsondump() const;
    std::string json_dumps() const;
    static AxisFeature jsonload(const nlohmann::ordered_json& data);
    static AxisFeature json_loads(const std::string& s);

    std::string pb_dumps() const;
    static AxisFeature pb_loads(const std::string& data);

private:
    mutable std::string _guid;
};

/**
 * @brief CollisionFeature — BVH / OBB / AABB collision pair between two elements.
 */
struct CollisionFeature {
    std::string mode = "bvh";  ///< "bvh", "obb", "aabb", ...

    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

    bool operator==(const CollisionFeature& o) const;
    bool operator!=(const CollisionFeature& o) const { return !(*this == o); }
    std::string str() const;
    std::string repr() const;

    nlohmann::ordered_json jsondump() const;
    std::string json_dumps() const;
    static CollisionFeature jsonload(const nlohmann::ordered_json& data);
    static CollisionFeature json_loads(const std::string& s);

    std::string pb_dumps() const;
    static CollisionFeature pb_loads(const std::string& data);

private:
    mutable std::string _guid;
};

/**
 * @brief Sum type of everything that can live on a graph edge as a structured
 * feature. `std::monostate` is the "no feature attached" alternative — it's a
 * standard empty placeholder type designed for use as a default variant arm.
 * Each feature family is its own variant arm with its own honest field set.
 */
using EdgeFeature = std::variant<
    std::monostate,
    CrossFeature,
    FaceFeature,
    AxisFeature,
    CollisionFeature
>;

/// Serialize an EdgeFeature to a pb byte string. Returns empty for monostate.
std::string edge_feature_pb_dumps(const EdgeFeature& f);

/// Deserialize an EdgeFeature from a pb byte string. Returns monostate on empty/unknown.
EdgeFeature edge_feature_pb_loads(const std::string& data);

/// Serialize an EdgeFeature to JSON. Returns {"kind":"Empty"} for monostate.
nlohmann::ordered_json edge_feature_jsondump(const EdgeFeature& f);

/// Deserialize an EdgeFeature from JSON. Returns monostate on null/unknown.
EdgeFeature edge_feature_jsonload(const nlohmann::ordered_json& data);

/// Extract the GUID from any non-empty EdgeFeature. Returns empty string for monostate.
std::string edge_feature_guid(const EdgeFeature& f);

} // namespace session_cpp
