#pragma once
#include "fmt/core.h"
#include "guid.h"
#include "json.h"
#include <algorithm>
#include <array>
#include <ostream>
#include <string>
#include <vector>

namespace session_proto {
class Color;
}

namespace session_cpp {

/// A named color with RGBA components in [0.0, 1.0].
class Color {
private:
    mutable std::string _guid; // Lazily minted GUID.

public:
    std::string name = "my_color"; // Color name.
    float r; // Red component.
    float g; // Green component.
    float b; // Blue component.
    float a; // Alpha component.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from RGBA components, each clamped to [0.0, 1.0].
    Color(float r = 0.94f, float g = 0.94f, float b = 0.94f, float a = 1.0f, std::string name = "my_color");

    /// Copy constructor (new guid, same data).
    Color(const Color& other);

    /// Copy assignment (new guid, same data).
    Color& operator=(const Color& other);

    /// Move keeps the guid so `return x;` does not fall back to the guid-minting copy.
    Color(Color&& other) noexcept = default;

    /// Move assignment keeps the guid.
    Color& operator=(Color&& other) noexcept = default;

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy GUID has been created.
    bool has_guid() const {
        return !_guid.empty();
    }

    /// Return the GUID, creating it on first access.
    const std::string& guid() const;

    /// Return a mutable GUID, creating it on first access.
    std::string& guid();

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Component by index (0=r, 1=g, 2=b, 3=a).
    float& operator[](int index);

    /// Component by index, read-only.
    const float& operator[](int index) const;

    /// Compare names and RGBA components.
    bool operator==(const Color& other) const;

    /// Return whether names or RGBA components differ.
    bool operator!=(const Color& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Presets
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return opaque white.
    static Color white();

    /// Return opaque black.
    static Color black();

    /// Return opaque grey.
    static Color grey();

    /// Return opaque red.
    static Color red();

    /// Return opaque orange.
    static Color orange();

    /// Return opaque yellow.
    static Color yellow();

    /// Return opaque lime.
    static Color lime();

    /// Return opaque green.
    static Color green();

    /// Return opaque mint.
    static Color mint();

    /// Return opaque cyan.
    static Color cyan();

    /// Return opaque azure.
    static Color azure();

    /// Return opaque blue.
    static Color blue();

    /// Return opaque violet.
    static Color violet();

    /// Return opaque magenta.
    static Color magenta();

    /// Return opaque pink.
    static Color pink();

    /// Return opaque maroon.
    static Color maroon();

    /// Return opaque brown.
    static Color brown();

    /// Return opaque olive.
    static Color olive();

    /// Return opaque teal.
    static Color teal();

    /// Return opaque navy.
    static Color navy();

    /// Return opaque purple.
    static Color purple();

    /// Return opaque silver.
    static Color silver();

    /// Return opaque light grey, the default surface color of meshes, breps and surfaces.
    static Color lightgrey();

    /// Return the 12 spectral colors in order.
    static std::vector<Color> palette();

    // ═══════════════════════════════════════════════════════════════════════════
    // Conversion
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the components as [r, g, b, a].
    std::array<float, 4> to_unified_array() const;

    /// Construct from [r, g, b, a].
    static Color from_unified_array(std::array<float, 4> arr);

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to an ordered JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Color jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static Color file_json_loads(const std::string& json_string);

    /// Write JSON to a file.
    void file_json_dump(const std::string& filename) const;

    /// Read JSON from a file.
    static Color file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    session_proto::Color to_proto() const;

    /// Construct from the protobuf message.
    static Color from_proto(const session_proto::Color& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static Color pb_loads(const std::string& data);

    /// Write protobuf bytes to a file.
    void pb_dump(const std::string& filename) const;

    /// Read protobuf bytes from a file.
    static Color pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "r, g, b, a".
    std::string str() const;

    /// Return "Color(name, r, g, b, a)".
    std::string repr() const;
};

/// Write the string representation to a stream.
std::ostream& operator<<(std::ostream& os, const Color& color);

} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::Color> {
    constexpr fmt::format_parse_context::iterator parse(fmt::format_parse_context& ctx) {
        return ctx.begin();
    }

    fmt::format_context::iterator format(const session_cpp::Color& color, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", color.str());
    }
};
