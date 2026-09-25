#pragma once
#include "color.h"
#include "element.h"
#include "guid.h"
#include "json.h"
#include "xform.h"
#include "fmt/core.h"
#include <cstdint>
#include <ostream>
#include <string>

namespace session_proto {
class InstanceRef;
}

namespace session_cpp {

/// A block reference: places a definition (by guid) at a transform.
class InstanceRef {
private:
    mutable std::string _guid; // Lazily minted GUID.

public:
    static constexpr uint32_t FLAG_HIDDEN = 1; // Not drawn.
    static constexpr uint32_t FLAG_LOCKED = 2; // Not selectable.
    static constexpr uint32_t FLAG_COLOR = 4; // color overrides the definition's.

    std::string name = "my_instance_ref"; // Instance name.
    std::string definition_guid; // Guid of the referenced definition.
    Xform xform; // Placement outside a Session; inside one Session::xforms places the instance and this stays identity.
    Color color = Color::white(); // Display color, used when flags has FLAG_COLOR.
    uint32_t flags = 0; // FLAG_* bits.
    std::vector<ElementFeature> features; // Per-instance features in the definition frame, drawn after the definition's own.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty reference.
    InstanceRef() {}

    /// Construct from a definition guid and a placement.
    InstanceRef(const std::string& definition_guid, const Xform& xform)
        : definition_guid(definition_guid), xform(xform) {}

    /// Copy with a new guid and the same data.
    InstanceRef(const InstanceRef& other);

    /// Copy-assign with a new guid and the same data.
    InstanceRef& operator=(const InstanceRef& other);

    /// Move while preserving the guid.
    InstanceRef(InstanceRef&& other) noexcept = default;

    /// Move-assign while preserving the guid.
    InstanceRef& operator=(InstanceRef&& other) noexcept = default;

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
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from a name, a definition guid and a placement.
    static InstanceRef with_name(const std::string& name, const std::string& definition_guid, const Xform& xform);

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the mutable placement matrix entry by index (0..15, column-major).
    double& operator[](int index);

    /// Return the placement matrix entry by index (0..15, column-major).
    const double& operator[](int index) const;

    /// Compare definition guid, placement, color, flags and features.
    bool operator==(const InstanceRef& other) const;

    /// Compare definition guid, placement, color, flags and features.
    bool operator!=(const InstanceRef& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Compose in place: xform = t * xform.
    void transform(const Xform& t);

    /// Return a composed copy.
    InstanceRef transformed(const Xform& t) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object; missing flags and features default to none.
    static InstanceRef jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static InstanceRef file_json_loads(const std::string& json_string);

    /// Write to a JSON file.
    void file_json_dump(const std::string& filename) const;

    /// Read from a JSON file.
    static InstanceRef file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message; an identity xform, and color without FLAG_COLOR, are not written.
    session_proto::InstanceRef to_proto() const;

    /// Construct from the protobuf message; an absent xform is identity and an absent color white.
    static InstanceRef from_proto(const session_proto::InstanceRef& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static InstanceRef pb_loads(const std::string& data);

    /// Write to a protobuf file.
    void pb_dump(const std::string& filename) const;

    /// Read from a protobuf file.
    static InstanceRef pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "definition_guid @ [tx, ty, tz]".
    std::string str() const;

    /// Return "InstanceRef(name, definition_guid, Color(...), flags)".
    std::string repr() const;
};

/// Write the instance string to a stream.
std::ostream& operator<<(std::ostream& os, const InstanceRef& ref);

} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::InstanceRef> {
    constexpr fmt::format_parse_context::iterator parse(fmt::format_parse_context& ctx) {
        return ctx.begin();
    }

    fmt::format_context::iterator format(const session_cpp::InstanceRef& ref, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", ref.str());
    }
};
