#include "interaction.h"
#include "interaction.pb.h"
#include <fstream>
#include <iterator>
#include <map>
#include <stdexcept>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Hex encoding
// ═══════════════════════════════════════════════════════════════════════════
/// Encode bytes as hex text, since interaction_data is opaque and JSON carries no bytes.
static std::string to_hex(const std::string& bytes) {

    static const std::string digits = "0123456789abcdef";
    std::string out;
    out.reserve(bytes.size() * 2);

    for (unsigned char c : bytes) {
        out += digits[c >> 4];
        out += digits[c & 15];
    }

    return out;
}

/// Decode hex text back to bytes.
static std::string from_hex(const std::string& hex) {

    std::string out;
    out.reserve(hex.size() / 2);

    for (size_t i = 0; i + 1 < hex.size(); i += 2)
        out += static_cast<char>(std::stoi(hex.substr(i, 2), nullptr, 16));

    return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// Registry lookup
// ═══════════════════════════════════════════════════════════════════════════
/// Function-local so a package registering from a static initializer finds it built.
static std::map<std::string, std::function<std::shared_ptr<Interaction>(const std::string& data)>>& interaction_registry() {

    static std::map<std::string, std::function<std::shared_ptr<Interaction>(const std::string& data)>> registry;

    return registry;
}

/// The registered subclass built from its data, an InteractionUnknown when the type is unknown or its factory fails, then given the guid and the name.
static std::shared_ptr<Interaction> build_registered(
    const std::string& type_name,
    const std::string& data,
    const std::string& guid,
    const std::string& name
) {

    const auto it = interaction_registry().find(type_name);
    std::shared_ptr<Interaction> interaction;

    if (it != interaction_registry().end()) {
        try {
            interaction = it->second(data);
        } catch (const std::exception&) {
            interaction = nullptr;
        }
    }

    if (!interaction)
        interaction = std::make_shared<InteractionUnknown>(type_name, data);

    if (!guid.empty())
        interaction->guid() = guid;

    interaction->name = name;

    return interaction;
}

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
Interaction::Interaction(std::string name)
    : name(std::move(name)) {}

Interaction::Interaction(const Interaction& other)
    : _guid(other.guid()), name(other.name) {}

Interaction& Interaction::operator=(const Interaction& other) {

    if (this == &other)
        return *this;

    _guid = other.guid();
    name = other.name;

    return *this;
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
bool Interaction::operator==(const Interaction& other) const {

    return name == other.name && interaction_type_name() == other.interaction_type_name() &&
        interaction_data_dumps() == other.interaction_data_dumps();
}

bool Interaction::operator!=(const Interaction& other) const {
    return !(*this == other);
}

// ═══════════════════════════════════════════════════════════════════════════
// Polymorphic registry
// ═══════════════════════════════════════════════════════════════════════════
void Interaction::register_type(
    const std::string& type_name,
    std::function<std::shared_ptr<Interaction>(const std::string& data)> factory
) {

    if (type_name.empty() || !factory)
        return;

    interaction_registry()[type_name] = std::move(factory);
}

bool Interaction::is_registered(const std::string& type_name) {
    return interaction_registry().count(type_name) > 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════
nlohmann::ordered_json Interaction::jsondump() const {

    return nlohmann::ordered_json{
        {"guid", guid()},
        {"interaction_data", to_hex(interaction_data_dumps())},
        {"interaction_type", interaction_type_name()},
        {"name", name},
        {"type", "Interaction"},
    };
}

std::shared_ptr<Interaction> Interaction::jsonload(const nlohmann::json& data) {

    return build_registered(
        data.value("interaction_type", std::string()),
        from_hex(data.value("interaction_data", std::string())),
        data.value("guid", std::string()),
        data.value("name", std::string())
    );
}

std::string Interaction::file_json_dumps() const {
    return jsondump().dump();
}

std::shared_ptr<Interaction> Interaction::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Interaction::file_json_dump(const std::string& filename) const {

    std::ofstream file(filename);
    file << jsondump().dump(4);
}

std::shared_ptr<Interaction> Interaction::file_json_load(const std::string& filename) {

    std::ifstream file(filename);

    return jsonload(nlohmann::json::parse(file));
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════
session_proto::Interaction Interaction::to_proto() const {

    session_proto::Interaction proto;
    proto.set_guid(guid());
    proto.set_name(name);
    proto.set_interaction_type(interaction_type_name());
    proto.set_interaction_data(interaction_data_dumps());

    return proto;
}

std::shared_ptr<Interaction> Interaction::from_proto(const session_proto::Interaction& proto) {
    return build_registered(proto.interaction_type(), proto.interaction_data(), proto.guid(), proto.name());
}

std::string Interaction::pb_dumps() const {
    return to_proto().SerializeAsString();
}

std::shared_ptr<Interaction> Interaction::pb_loads(const std::string& data) {

    session_proto::Interaction proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse Interaction protobuf data");

    return from_proto(proto);
}

void Interaction::pb_dump(const std::string& filename) const {

    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

std::shared_ptr<Interaction> Interaction::pb_load(const std::string& filename) {

    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
std::string Interaction::str() const {
    return fmt::format("{}({})", interaction_type_name(), name);
}

std::string Interaction::repr() const {
    return fmt::format("{}({}, {})", interaction_type_name(), guid(), name);
}

// ═══════════════════════════════════════════════════════════════════════════
// InteractionUnknown
// ═══════════════════════════════════════════════════════════════════════════
InteractionUnknown::InteractionUnknown(std::string type_name, std::string data, std::string name)
    : Interaction(std::move(name)), type_name(std::move(type_name)), data(std::move(data)) {}

std::string InteractionUnknown::interaction_type_name() const {
    return type_name;
}

std::string InteractionUnknown::interaction_data_dumps() const {
    return data;
}

std::shared_ptr<Interaction> InteractionUnknown::clone() const {
    return std::make_shared<InteractionUnknown>(*this);
}

std::ostream& operator<<(std::ostream& os, const Interaction& interaction) {
    return os << interaction.str();
}

} // namespace session_cpp
