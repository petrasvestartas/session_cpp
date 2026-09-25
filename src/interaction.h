#pragma once
#include "fmt/core.h"
#include "guid.h"
#include "json.h"
#include <functional>
#include <memory>
#include <ostream>
#include <string>

namespace session_proto {
class Interaction;
}

namespace session_cpp {

/// What joins two elements, stored on their graph edge: an abstract base with a guid, a name and a registry that loads each subclass by its type name.
class Interaction {
private:
    mutable std::string _guid; // Lazily minted guid.

public:
    std::string name; // What joins the pair, e.g. "glue"; empty when unnamed.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from a name; the class is abstract, so only a subclass constructs one.
    Interaction(std::string name = "");

    /// Copy with the same guid, minting it on other first, so a stored interaction keeps its identity.
    Interaction(const Interaction& other);

    /// Copy-assign with the same guid, minting it on other first.
    Interaction& operator=(const Interaction& other);

    /// Move while preserving the guid.
    Interaction(Interaction&& other) noexcept = default;

    /// Move-assign while preserving the guid.
    Interaction& operator=(Interaction&& other) noexcept = default;

    /// Destroy the interaction.
    virtual ~Interaction() = default;

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

    /// Return the type name the subclass registered its factory under.
    virtual std::string interaction_type_name() const = 0;

    /// Return the subclass's own state, opaque to the kernel; its factory reads it back.
    virtual std::string interaction_data_dumps() const = 0;

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Compare name, type and data; guid ignored.
    bool operator==(const Interaction& other) const;

    /// Compare name, type and data; guid ignored.
    bool operator!=(const Interaction& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Utilities
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return a polymorphic copy of the same subclass with the same guid.
    virtual std::shared_ptr<Interaction> clone() const = 0;

    // ═══════════════════════════════════════════════════════════════════════════
    // Polymorphic registry
    // ═══════════════════════════════════════════════════════════════════════════
    /// Register factory for type_name, a factory building one subclass from its interaction_data; re-registering the same name replaces it.
    static void register_type(
        const std::string& type_name,
        std::function<std::shared_ptr<Interaction>(const std::string& data)> factory
    );

    /// Return whether a factory is registered for type_name.
    static bool is_registered(const std::string& type_name);

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object through the registry; an unregistered type loads as an InteractionUnknown.
    static std::shared_ptr<Interaction> jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string through the registry; an unregistered type loads as an InteractionUnknown.
    static std::shared_ptr<Interaction> file_json_loads(const std::string& json_string);

    /// Write to a JSON file.
    void file_json_dump(const std::string& filename) const;

    /// Read from a JSON file through the registry; an unregistered type loads as an InteractionUnknown.
    static std::shared_ptr<Interaction> file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    session_proto::Interaction to_proto() const;

    /// Construct from the protobuf message through the registry; an unregistered type loads as an InteractionUnknown.
    static std::shared_ptr<Interaction> from_proto(const session_proto::Interaction& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes through the registry; an unregistered type loads as an InteractionUnknown.
    static std::shared_ptr<Interaction> pb_loads(const std::string& data);

    /// Write to a protobuf file.
    void pb_dump(const std::string& filename) const;

    /// Read from a protobuf file through the registry; an unregistered type loads as an InteractionUnknown.
    static std::shared_ptr<Interaction> pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "Type(name)".
    virtual std::string str() const;

    /// Return "Type(guid, name)".
    virtual std::string repr() const;
};

/// An interaction whose type has no registered factory: a load keeps its type name and data so a save writes them back unchanged.
class InteractionUnknown : public Interaction {
public:
    std::string type_name; // The type name it was written under.
    std::string data; // Its opaque state.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from a type name, its data and a name.
    InteractionUnknown(std::string type_name = "", std::string data = "", std::string name = "");

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the type name it was written under.
    std::string interaction_type_name() const override;

    /// Return its opaque state.
    std::string interaction_data_dumps() const override;

    // ═══════════════════════════════════════════════════════════════════════════
    // Utilities
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return a copy with the same guid.
    std::shared_ptr<Interaction> clone() const override;
};

/// Write the interaction string to a stream.
std::ostream& operator<<(std::ostream& os, const Interaction& interaction);

} // namespace session_cpp
