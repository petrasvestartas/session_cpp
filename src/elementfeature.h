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
 * @brief CrossElementFeature — plane-to-face cross joint between plate elements.
 *
 * Geometry (joint_area, joint_lines, joint_volumes) lives in Session::lookup
 * as Polylines and is referenced here by GUID.
 */
struct CrossElementFeature {
    std::pair<int,int> face_ids_a{-1,-1};
    std::pair<int,int> face_ids_b{-1,-1};
    std::string joint_area_guid;
    std::array<std::string,2> joint_volume_guids;
    std::array<std::string,2> joint_line_guids;

    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

    bool operator==(const CrossElementFeature& o) const;
    bool operator!=(const CrossElementFeature& o) const { return !(*this == o); }
    std::string str() const;
    std::string repr() const;

    nlohmann::ordered_json jsondump() const;
    std::string file_json_dumps() const;
    static CrossElementFeature jsonload(const nlohmann::ordered_json& data);
    static CrossElementFeature file_json_loads(const std::string& s);

    std::string pb_dumps() const;
    static CrossElementFeature pb_loads(const std::string& data);

private:
    mutable std::string _guid;
};

/**
 * @brief FaceElementFeature — face-to-face coplanar joint between two elements.
 */
struct FaceElementFeature {
    int face_id_a = -1;
    int face_id_b = -1;
    std::string joint_area_guid;

    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

    bool operator==(const FaceElementFeature& o) const;
    bool operator!=(const FaceElementFeature& o) const { return !(*this == o); }
    std::string str() const;
    std::string repr() const;

    nlohmann::ordered_json jsondump() const;
    std::string file_json_dumps() const;
    static FaceElementFeature jsonload(const nlohmann::ordered_json& data);
    static FaceElementFeature file_json_loads(const std::string& s);

    std::string pb_dumps() const;
    static FaceElementFeature pb_loads(const std::string& data);

private:
    mutable std::string _guid;
};

/**
 * @brief AxisElementFeature — closest-polyline / axis-to-axis connection between
 * two axis-based elements (e.g. beams). Stores the GUIDs of the two axes,
 * the connector segment between them, and the minimum distance.
 */
struct AxisElementFeature {
    std::string axis_a_guid;
    std::string axis_b_guid;
    std::string connector_guid;
    double distance = 0.0;

    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

    bool operator==(const AxisElementFeature& o) const;
    bool operator!=(const AxisElementFeature& o) const { return !(*this == o); }
    std::string str() const;
    std::string repr() const;

    nlohmann::ordered_json jsondump() const;
    std::string file_json_dumps() const;
    static AxisElementFeature jsonload(const nlohmann::ordered_json& data);
    static AxisElementFeature file_json_loads(const std::string& s);

    std::string pb_dumps() const;
    static AxisElementFeature pb_loads(const std::string& data);

private:
    mutable std::string _guid;
};

/**
 * @brief CollisionElementFeature — SpatialBVH / OBB / AABB collision pair between two elements.
 */
struct CollisionElementFeature {
    std::string mode = "bvh";  ///< "bvh", "obb", "aabb", ...

    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

    bool operator==(const CollisionElementFeature& o) const;
    bool operator!=(const CollisionElementFeature& o) const { return !(*this == o); }
    std::string str() const;
    std::string repr() const;

    nlohmann::ordered_json jsondump() const;
    std::string file_json_dumps() const;
    static CollisionElementFeature jsonload(const nlohmann::ordered_json& data);
    static CollisionElementFeature file_json_loads(const std::string& s);

    std::string pb_dumps() const;
    static CollisionElementFeature pb_loads(const std::string& data);

private:
    mutable std::string _guid;
};

/**
 * @brief Sum type of everything that can live on a graph edge as a structured
 * element feature. `std::monostate` is the "no feature attached" alternative — it's a
 * standard empty placeholder type designed for use as a default variant arm.
 * Each element feature family is its own variant arm with its own honest field set.
 */
using EdgeElementFeature = std::variant<
    std::monostate,
    CrossElementFeature,
    FaceElementFeature,
    AxisElementFeature,
    CollisionElementFeature
>;

/// Serialize an EdgeElementFeature to a pb byte string. Returns empty for monostate.
std::string edge_elementfeature_pb_dumps(const EdgeElementFeature& f);

/// Deserialize an EdgeElementFeature from a pb byte string. Returns monostate on empty/unknown.
EdgeElementFeature edge_elementfeature_pb_loads(const std::string& data);

/// Serialize an EdgeElementFeature to JSON. Returns {"kind":"Empty"} for monostate.
nlohmann::ordered_json edge_elementfeature_jsondump(const EdgeElementFeature& f);

/// Deserialize an EdgeElementFeature from JSON. Returns monostate on null/unknown.
EdgeElementFeature edge_elementfeature_jsonload(const nlohmann::ordered_json& data);

/// Extract the GUID from any non-empty EdgeElementFeature. Returns empty string for monostate.
std::string edge_elementfeature_guid(const EdgeElementFeature& f);

} // namespace session_cpp
