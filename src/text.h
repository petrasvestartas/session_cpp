#pragma once
#include "color.h"
#include "guid.h"
#include "json.h"
#include "point.h"
#include <string>

namespace session_proto {
class Component;
}

namespace session_cpp {

class Component;

/// Text or TeX with a position, pixel height, colour, outline and projection mode.
class Text {
private:
    mutable std::string _guid; // Lazily minted GUID.

public:
    std::string name = "my_text"; // Text name.
    std::string text; // Plain text or TeX source.
    Point at; // Position in the text coordinate system.
    double height = 28.0; // Letter height in CSS pixels.
    Color color = Color::white(); // Text colour.
    double outline = 0.0; // Glyph outline width in CSS pixels; negative is black, positive white.
    bool math = false; // Whether text contains TeX.
    bool spatial = false; // Whether the text belongs to the navigable 3D region.
    bool serif = false; // Whether to use the serif mathematics face.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct empty text.
    Text() = default;

    /// Construct text at a position.
    Text(const std::string& text, const Point& at = Point());

    /// Construct TeX with flat-panel or fixed-size spatial defaults.
    static Text from_tex(const std::string& text, const Point& at = Point(), bool spatial = false);

    /// Copy with a new guid and the same data.
    Text(const Text& other);

    /// Copy-assign with a new guid and the same data.
    Text& operator=(const Text& other);

    /// Move while preserving the guid.
    Text(Text&& other) noexcept = default;

    /// Move-assign while preserving the guid.
    Text& operator=(Text&& other) noexcept = default;

    /// Copy with a new guid and the same data.
    Text duplicate() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    bool has_guid() const {
        return !_guid.empty();
    }

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

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Compare every text property, ignoring the guid.
    bool operator==(const Text& other) const;

    /// Return whether two texts differ.
    bool operator!=(const Text& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to an ordered JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Text jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static Text file_json_loads(const std::string& json_string);

    /// Write JSON to a file.
    void file_json_dump(const std::string& filename) const;

    /// Read JSON from a file.
    static Text file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the generic component protobuf message.
    session_proto::Component to_proto() const;

    /// Construct from a generic component protobuf message.
    static Text from_proto(const session_proto::Component& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static Text pb_loads(const std::string& data);

    /// Write protobuf bytes to a file.
    void pb_dump(const std::string& filename) const;

    /// Read protobuf bytes from a file.
    static Text pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Component
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return a Session component of type Text with this guid and name.
    Component to_component() const;

    /// Construct from a component made by to_component.
    static Text from_component(const Component& component);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the text content.
    std::string str() const;

    /// Return a representation with every property.
    std::string repr() const;
};

} // namespace session_cpp
