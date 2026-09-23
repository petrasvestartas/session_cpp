#include "instance_ref.h"
#include "tolerance.h"
#include "instance_ref.pb.h"
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
InstanceRef::InstanceRef(const InstanceRef& other)
    : name(other.name), definition_guid(other.definition_guid), xform(other.xform), color(other.color), flags(other.flags),
      features(other.features) {}

InstanceRef& InstanceRef::operator=(const InstanceRef& other) {

    if (this == &other)
        return *this;

    _guid.clear();
    name = other.name;
    definition_guid = other.definition_guid;
    xform = other.xform;
    color = other.color;
    flags = other.flags;
    features = other.features;

    return *this;
}

// ═══════════════════════════════════════════════════════════════════════════
// Accessors
// ═══════════════════════════════════════════════════════════════════════════
const std::string& InstanceRef::guid() const {

    if (_guid.empty())
        _guid = ::guid();

    return _guid;
}

std::string& InstanceRef::guid() {

    if (_guid.empty())
        _guid = ::guid();

    return _guid;
}

// ═══════════════════════════════════════════════════════════════════════════
// Static constructors
// ═══════════════════════════════════════════════════════════════════════════
InstanceRef InstanceRef::with_name(const std::string& name, const std::string& definition_guid, const Xform& xform) {

    InstanceRef ref(definition_guid, xform);
    ref.name = name;

    return ref;
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
double& InstanceRef::operator[](int index) {

    if (index < 0 || index >= 16)
        throw std::out_of_range("Index out of bounds");

    return xform.m[index];
}

const double& InstanceRef::operator[](int index) const {

    if (index < 0 || index >= 16)
        throw std::out_of_range("Index out of bounds");

    return xform.m[index];
}

bool InstanceRef::operator==(const InstanceRef& other) const {
    return definition_guid == other.definition_guid && xform == other.xform && color == other.color && flags == other.flags &&
        features == other.features;
}

bool InstanceRef::operator!=(const InstanceRef& other) const { return !(*this == other); }

// ═══════════════════════════════════════════════════════════════════════════
// Transformation
// ═══════════════════════════════════════════════════════════════════════════
void InstanceRef::transform(const Xform& t) { xform = t * xform; }

InstanceRef InstanceRef::transformed(const Xform& t) const {

    InstanceRef result = *this;
    result.transform(t);

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════
nlohmann::ordered_json InstanceRef::jsondump() const {

    nlohmann::ordered_json feats = nlohmann::ordered_json::array();

    for (const ElementFeature& feature : features)
        feats.push_back(feature.jsondump());

    nlohmann::ordered_json data;
    data["color"] = color.jsondump();
    data["definition_guid"] = definition_guid;
    data["features"] = feats;
    data["flags"] = flags;
    data["guid"] = guid();
    data["name"] = name;
    data["type"] = "InstanceRef";
    data["xform"] = xform.jsondump();

    return data;
}

InstanceRef InstanceRef::jsonload(const nlohmann::json& data) {

    InstanceRef ref(data["definition_guid"], Xform::jsonload(data["xform"]));
    ref.color = Color::jsonload(data["color"]);
    ref.flags = data.value("flags", 0u);
    ref.guid() = data["guid"];
    ref.name = data["name"];

    for (const nlohmann::json& feature : data.value("features", nlohmann::json::array()))
        ref.features.push_back(ElementFeature::jsonload(feature));

    return ref;
}

std::string InstanceRef::file_json_dumps() const { return jsondump().dump(); }

InstanceRef InstanceRef::file_json_loads(const std::string& json_string) { return jsonload(nlohmann::ordered_json::parse(json_string)); }

void InstanceRef::file_json_dump(const std::string& filename) const {

    std::ofstream file(filename);
    file << jsondump().dump(4);
}

InstanceRef InstanceRef::file_json_load(const std::string& filename) {

    std::ifstream file(filename);

    return jsonload(nlohmann::json::parse(file));
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════
session_proto::InstanceRef InstanceRef::to_proto() const {

    session_proto::InstanceRef proto;

    if (has_guid())
        proto.set_guid(guid());

    proto.set_name(name);
    proto.set_definition_guid(definition_guid);

    if (!xform.is_identity())
        *proto.mutable_xform() = xform.to_proto();

    if (flags & FLAG_COLOR)
        *proto.mutable_color() = color.to_proto();

    proto.set_flags(flags);

    for (const ElementFeature& feature : features)
        *proto.add_features() = feature.to_proto();

    return proto;
}

InstanceRef InstanceRef::from_proto(const session_proto::InstanceRef& proto) {

    InstanceRef ref;

    if (!proto.guid().empty())
        ref.guid() = proto.guid();

    ref.name = proto.name();
    ref.definition_guid = proto.definition_guid();

    if (proto.has_xform())
        ref.xform = Xform::from_proto(proto.xform());

    if (proto.has_color())
        ref.color = Color::from_proto(proto.color());

    ref.flags = proto.flags();

    for (const session_proto::ElementFeature& feature : proto.features())
        ref.features.push_back(ElementFeature::from_proto(feature));

    return ref;
}

std::string InstanceRef::pb_dumps() const { return to_proto().SerializeAsString(); }

InstanceRef InstanceRef::pb_loads(const std::string& data) {

    session_proto::InstanceRef proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse InstanceRef protobuf data");

    return from_proto(proto);
}

void InstanceRef::pb_dump(const std::string& filename) const {

    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

InstanceRef InstanceRef::pb_load(const std::string& filename) {

    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
std::string InstanceRef::str() const {

    const int prec = Tolerance::ROUNDING;

    return fmt::format(
        "{} @ [{}, {}, {}]",
        definition_guid,
        TOLERANCE.format_number(xform.m[12], prec),
        TOLERANCE.format_number(xform.m[13], prec),
        TOLERANCE.format_number(xform.m[14], prec)
    );
}

std::string InstanceRef::repr() const { return fmt::format("InstanceRef({}, {}, {}, {})", name, definition_guid, color.repr(), flags); }

std::ostream& operator<<(std::ostream& os, const InstanceRef& ref) { return os << ref.str(); }

} // namespace session_cpp
