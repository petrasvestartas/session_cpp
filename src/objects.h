#pragma once
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
#include <fstream>
#include <memory>
#include <string>
#include <variant>
#include <vector>

namespace session_cpp {

/// A custom domain object stored generically in a Session; every field except type/guid/name lives in extra.
struct Component {
    std::string type_name;             // Class name, e.g. "FloorBuilder".
    std::string name = "my_component"; // Human-readable name.
    nlohmann::ordered_json extra;      // All custom fields.

    /// Return whether the lazy guid has been created.
    bool has_guid() const { return !_guid.empty(); }

    /// Return the guid, creating it on first access.
    const std::string& guid() const {
        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return the mutable guid, creating it on first access.
    std::string& guid() {
        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Serialize to a JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Component jsonload(const nlohmann::json& data);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static Component pb_loads(const std::string& data);

private:
    mutable std::string _guid; // Lazily minted guid.
};

/// A collection of geometry objects.
class Objects {
public:
    std::string name = "my_objects"; // The name of the collection.
    std::shared_ptr<std::vector<std::shared_ptr<Point>>> points; // Points.
    std::shared_ptr<std::vector<std::shared_ptr<Line>>> lines; // Lines.
    std::shared_ptr<std::vector<std::shared_ptr<Plane>>> planes; // Planes.
    std::shared_ptr<std::vector<std::shared_ptr<OBB>>> bboxes; // Bounding boxes.
    std::shared_ptr<std::vector<std::shared_ptr<Polyline>>> polylines; // Polylines.
    std::shared_ptr<std::vector<std::shared_ptr<PointCloud>>> pointclouds; // Point clouds.
    std::shared_ptr<std::vector<std::shared_ptr<Mesh>>> meshes; // Meshes.
    std::shared_ptr<std::vector<std::shared_ptr<NurbsCurve>>> nurbscurves; // NURBS curves.
    std::shared_ptr<std::vector<std::shared_ptr<NurbsSurface>>> nurbssurfaces; // NURBS surfaces.
    std::shared_ptr<std::vector<std::shared_ptr<BRep>>> breps; // BReps.
    std::shared_ptr<std::vector<std::shared_ptr<Element>>> elements; // Elements.
    std::shared_ptr<std::vector<Component>> components; // Components.

    /// Construct an empty collection with every list allocated.
    Objects(std::string name = "my_objects");

    /// Copy every list and every object in it, guids included, so a Session's indexes still match.
    Objects(const Objects& other);

    /// Copy-assign every list and every object in it, guids included.
    Objects& operator=(const Objects& other);

    /// Move the lists as they are.
    Objects(Objects&&) noexcept = default;

    /// Move-assign the lists as they are.
    Objects& operator=(Objects&&) noexcept = default;

    /// Return whether the lazy guid has been created.
    bool has_guid() const { return !_guid.empty(); }

    /// Return the guid, creating it on first access.
    const std::string& guid() const {
        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return the mutable guid, creating it on first access.
    std::string& guid() {
        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return a string representation of the collection.
    std::string str() const;

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

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static Objects pb_loads(const std::string& data);

    /// Write to a protobuf file.
    void pb_dump(const std::string& filename) const;

    /// Read from a protobuf file.
    static Objects pb_load(const std::string& filename);

private:
    mutable std::string _guid; // Lazily minted guid.
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

/// Anything an Objects collection holds: geometry, or a Component.
using Item = std::variant<Geometry, Component>;

/// Write the collection string to a stream.
std::ostream& operator<<(std::ostream& os, const Objects& objects);

} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::Objects> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    auto format(const session_cpp::Objects& o, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", o.str());
    }
};
