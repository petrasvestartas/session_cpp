#pragma once
#include "color.h"
#include "xform.h"
#include "fmt/core.h"
#include "guid.h"
#include "json.h"
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace session_cpp {

/**
 * @class InstanceRef
 * @brief A block reference: places a definition (by guid) at a transform.
 *
 * The only per-instance data is the placement `xform`; the geometry lives once
 * in the definition the `definition_guid` points to. Mirrors the Rhino block model.
 */
class InstanceRef {
public:
    std::string name = "my_instance_ref";   ///< Instance identifier/name
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string definition_guid;             ///< Guid of the definition this instance places
    Xform xform;                             ///< Placement transform (default identity)
    Color color = Color::white();            ///< Per-instance color override
    uint32_t flags = 0;                      ///< Reserved: selection / cull / visibility

private:
    mutable std::string _guid;               ///< Lazily generated unique identifier

public:

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Default constructor.
    InstanceRef();

    /// Constructor from a definition guid and a placement transform.
    InstanceRef(const std::string& definition_guid, const Xform& xform);

    /// Constructor with name, definition guid, and placement transform.
    static InstanceRef with_name(const std::string& name, const std::string& definition_guid, const Xform& xform);

    /// Copy constructor (creates a new guid while copying data)
    InstanceRef(const InstanceRef& other);

    /// Copy assignment (creates a new guid while copying data)
    InstanceRef& operator=(const InstanceRef& other);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Operators
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Subscript into the placement matrix (0..15, column-major).
    double& operator[](int index);

    /// Subscript into the placement matrix (const).
    const double& operator[](int index) const;

    bool operator==(const InstanceRef& other) const;
    bool operator!=(const InstanceRef& other) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Compose an extra transform onto the placement (in-place): xform = t * xform.
    void transform(const Xform& t);

    /// Return a copy with an extra transform composed onto the placement.
    InstanceRef transformed(const Xform& t) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // String Representation
    ///////////////////////////////////////////////////////////////////////////////////////////

    std::string str() const;    ///< simple string (definition + placement translation)
    std::string repr() const;   ///< detailed representation

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    nlohmann::ordered_json jsondump() const;
    static InstanceRef jsonload(const nlohmann::json& data);
    void file_json_dump(const std::string& filename) const;
    static InstanceRef file_json_load(const std::string& filename);
    std::string file_json_dumps() const;
    static InstanceRef file_json_loads(const std::string& json_string);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    std::string pb_dumps() const;
    static InstanceRef pb_loads(const std::string& data);
    void pb_dump(const std::string& filename) const;
    static InstanceRef pb_load(const std::string& filename);
};

/// Output stream operator for InstanceRef.
std::ostream& operator<<(std::ostream& os, const InstanceRef& ref);

}  // namespace session_cpp

// fmt formatter specialization for InstanceRef
template <> struct fmt::formatter<session_cpp::InstanceRef> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  auto format(const session_cpp::InstanceRef& o, fmt::format_context& ctx) const {
    return fmt::format_to(ctx.out(), "{}", o.str());
  }
};
