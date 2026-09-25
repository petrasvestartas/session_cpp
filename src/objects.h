#pragma once
#include "collection.h"
#include "guid.h"
#include "json.h"
#include "point.h"
#include "vector.h"
#include "line.h"
#include "plane.h"
#include "obb.h"
#include "polyline.h"
#include "pointcloud.h"
#include "mesh.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "brep.h"
#include "element.h"
#include "instance_ref.h"
#include "fmt/core.h"
#include <fstream>
#include <memory>
#include <ostream>
#include <string>
#include <variant>
#include <vector>

namespace session_proto {
class Component;
class Objects;
}

namespace session_cpp {

/// A custom domain object stored generically in a Session; every field except type/guid/name lives in extra.
class Component {
private:
    mutable std::string _guid; // Lazily minted GUID.

public:
    std::string type_name; // Class name, e.g. "FloorBuilder".
    std::string name = "my_component"; // Human-readable name.
    nlohmann::ordered_json extra; // All custom fields.

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    bool has_guid() const {
        return !_guid.empty();
    }

    /// Return the guid, creating it on first access.
    const std::string& guid() const;

    /// Return the mutable guid, creating it on first access.
    std::string& guid();

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Component jsonload(const nlohmann::json& data);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    session_proto::Component to_proto() const;

    /// Construct from the protobuf message.
    static Component from_proto(const session_proto::Component& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static Component pb_loads(const std::string& data);
};

/// A collection of geometry objects.
class Objects {
private:
    mutable std::string _guid; // Lazily minted GUID.

public:
    std::string name = "my_objects"; // The name of the collection.
    std::shared_ptr<Collection<std::shared_ptr<Point>>> points; // Points.
    std::shared_ptr<Collection<std::shared_ptr<Line>>> lines; // Lines.
    std::shared_ptr<Collection<std::shared_ptr<Plane>>> planes; // Planes.
    std::shared_ptr<Collection<std::shared_ptr<OBB>>> bboxes; // Bounding boxes.
    std::shared_ptr<Collection<std::shared_ptr<Polyline>>> polylines; // Polylines.
    std::shared_ptr<Collection<std::shared_ptr<PointCloud>>> pointclouds; // Point clouds.
    std::shared_ptr<Collection<std::shared_ptr<Mesh>>> meshes; // Meshes.
    std::shared_ptr<Collection<std::shared_ptr<NurbsCurve>>> nurbscurves; // NURBS curves.
    std::shared_ptr<Collection<std::shared_ptr<NurbsSurface>>> nurbssurfaces; // NURBS surfaces.
    std::shared_ptr<Collection<std::shared_ptr<BRep>>> breps; // BReps.
    std::shared_ptr<Collection<std::shared_ptr<Element>>> elements; // Elements.
    std::shared_ptr<Collection<Component>> components; // Components.
    std::shared_ptr<Collection<std::shared_ptr<InstanceRef>>> instances; // Instances, each placing a definition of Session::definitions by guid.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty collection with every list allocated.
    Objects(std::string name = "my_objects");

    /// Copy every list and every object in it, guids included, so a Session's indexes still match.
    Objects(const Objects& other);

    /// Copy-assign every list and every object in it, guids included.
    Objects& operator=(const Objects& other);

    /// Move the lists as they are.
    Objects(Objects&& other) noexcept = default;

    /// Move-assign the lists as they are.
    Objects& operator=(Objects&& other) noexcept = default;

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    bool has_guid() const {
        return !_guid.empty();
    }

    /// Return the guid, creating it on first access.
    const std::string& guid() const;

    /// Return the mutable guid, creating it on first access.
    std::string& guid();

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Objects jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static Objects file_json_loads(const std::string& json_string);

    /// Write to a JSON file.
    void file_json_dump(const std::string& filename) const;

    /// Read from a JSON file.
    static Objects file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    session_proto::Objects to_proto() const;

    /// Construct from the protobuf message; elements load through the polymorphic registry.
    static Objects from_proto(const session_proto::Objects& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static Objects pb_loads(const std::string& data);

    /// Write to a protobuf file.
    void pb_dump(const std::string& filename) const;

    /// Read from a protobuf file.
    static Objects pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "Objects(name=..., guid=..., points=...)".
    std::string str() const;

    /// Return "Objects(name=..., guid=..., points=...)".
    std::string repr() const;
};

/// All geometry types as a variant; a new type joins here and in the checklist at the top of session.cpp.
using Geometry = std::variant<
    std::shared_ptr<OBB>,
    std::shared_ptr<Line>,
    std::shared_ptr<Mesh>,
    std::shared_ptr<Plane>,
    std::shared_ptr<Point>,
    std::shared_ptr<PointCloud>,
    std::shared_ptr<NurbsCurve>,
    std::shared_ptr<NurbsSurface>,
    std::shared_ptr<Polyline>,
    std::shared_ptr<BRep>,
    std::shared_ptr<Element>>;

/// Anything an Objects collection holds: geometry, a Component or an InstanceRef.
using Item = std::variant<Geometry, Component, std::shared_ptr<InstanceRef>>;

/// Write the collection string to a stream.
std::ostream& operator<<(std::ostream& os, const Objects& objects);

} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::Objects> {
    constexpr fmt::format_parse_context::iterator parse(fmt::format_parse_context& ctx) {
        return ctx.begin();
    }

    fmt::format_context::iterator format(const session_cpp::Objects& objects, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", objects.str());
    }
};
