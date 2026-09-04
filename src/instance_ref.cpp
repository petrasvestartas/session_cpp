#include "instance_ref.h"
#include "tolerance.h"
#include <fstream>
#include <stdexcept>

#include "instance_ref.pb.h"

namespace session_cpp {

InstanceRef::InstanceRef() {}

InstanceRef::InstanceRef(const std::string& definition_guid, const Xform& xform)
    : definition_guid(definition_guid), xform(xform) {}

InstanceRef InstanceRef::with_name(const std::string& name, const std::string& definition_guid, const Xform& xform) {
    InstanceRef ref(definition_guid, xform);
    ref.name = name;
    return ref;
}

/// Copy constructor (creates a new guid() while copying data)
InstanceRef::InstanceRef(const InstanceRef& other)
    : name(other.name),
      definition_guid(other.definition_guid),
      xform(other.xform),
      color(other.color),
      flags(other.flags) {}

/// Copy assignment (creates a new guid() while copying data)
InstanceRef& InstanceRef::operator=(const InstanceRef& other) {
    if (this != &other) {
        _guid.clear();
        name = other.name;
        definition_guid = other.definition_guid;
        xform = other.xform;
        color = other.color;
        flags = other.flags;
    }
    return *this;
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════

double& InstanceRef::operator[](int index) {
    if (index < 0 || index >= 16) throw std::out_of_range("Index out of bounds");
    return xform.m[index];
}

const double& InstanceRef::operator[](int index) const {
    if (index < 0 || index >= 16) throw std::out_of_range("Index out of bounds");
    return xform.m[index];
}

bool InstanceRef::operator==(const InstanceRef& other) const {
    return definition_guid == other.definition_guid
        && xform.m == other.xform.m
        && color == other.color
        && flags == other.flags;
}

bool InstanceRef::operator!=(const InstanceRef& other) const {
    return !(*this == other);
}

// ═══════════════════════════════════════════════════════════════════════════
// Transformation
// ═══════════════════════════════════════════════════════════════════════════

void InstanceRef::transform(const Xform& t) {
    xform = t * xform;
}

InstanceRef InstanceRef::transformed(const Xform& t) const {
    InstanceRef result = *this;
    result.transform(t);
    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// String Representation
// ═══════════════════════════════════════════════════════════════════════════

std::string InstanceRef::str() const {
    int prec = static_cast<int>(Tolerance::ROUNDING);
    return fmt::format(
        "{} @ [{}, {}, {}]",
        definition_guid,
        TOLERANCE.format_number(xform.m[12], prec),
        TOLERANCE.format_number(xform.m[13], prec),
        TOLERANCE.format_number(xform.m[14], prec));
}

std::string InstanceRef::repr() const {
    return fmt::format(
        "InstanceRef({}, {}, Color({}, {}, {}, {}), {})",
        name,
        definition_guid,
        color.r, color.g, color.b, color.a,
        flags);
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════

nlohmann::ordered_json InstanceRef::jsondump() const {
    // Alphabetical order to match Rust's serde_json
    nlohmann::ordered_json data;
    data["color"] = color.jsondump();
    data["definition_guid"] = definition_guid;
    data["flags"] = flags;
    data["guid"] = guid();
    data["name"] = name;
    data["type"] = "InstanceRef";
    data["xform"] = xform.jsondump();
    return data;
}

InstanceRef InstanceRef::jsonload(const nlohmann::json& data) {
    InstanceRef ref;
    ref.color = Color::jsonload(data["color"]);
    ref.definition_guid = data["definition_guid"];
    ref.flags = data["flags"];
    ref.guid() = data["guid"];
    ref.name = data["name"];
    ref.xform = Xform::jsonload(data["xform"]);
    return ref;
}

std::string InstanceRef::file_json_dumps() const {
    return jsondump().dump();
}

InstanceRef InstanceRef::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void InstanceRef::file_json_dump(const std::string& filename) const {
    std::ofstream ofs(filename);
    ofs << jsondump().dump(4);
    ofs.close();
}

InstanceRef InstanceRef::file_json_load(const std::string& filename) {
    std::ifstream ifs(filename);
    nlohmann::json data = nlohmann::json::parse(ifs);
    ifs.close();
    return jsonload(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════

std::string InstanceRef::pb_dumps() const {
    session_proto::InstanceRef proto;
    proto.set_guid(guid());
    proto.set_name(name);
    proto.set_definition_guid(definition_guid);
    auto* proto_xform = proto.mutable_xform();
    proto_xform->set_name(xform.name);
    for (int i = 0; i < 16; ++i) {
        proto_xform->add_matrix(xform.m[i]);
    }
    auto* proto_color = proto.mutable_color();
    proto_color->set_r(color.r);
    proto_color->set_g(color.g);
    proto_color->set_b(color.b);
    proto_color->set_a(color.a);
    proto.set_flags(flags);
    return proto.SerializeAsString();
}

InstanceRef InstanceRef::pb_loads(const std::string& data) {
    session_proto::InstanceRef proto;
    proto.ParseFromString(data);
    InstanceRef ref;
    ref.guid() = proto.guid();
    ref.name = proto.name();
    ref.definition_guid = proto.definition_guid();
    if (proto.has_xform()) {
        ref.xform.name = proto.xform().name();
        for (int i = 0; i < proto.xform().matrix_size() && i < 16; ++i) {
            ref.xform.m[i] = proto.xform().matrix(i);
        }
    }
    if (proto.has_color()) {
        ref.color.r = proto.color().r();
        ref.color.g = proto.color().g();
        ref.color.b = proto.color().b();
        ref.color.a = proto.color().a();
    }
    ref.flags = proto.flags();
    return ref;
}

void InstanceRef::pb_dump(const std::string& filename) const {
    std::ofstream ofs(filename, std::ios::binary);
    ofs << pb_dumps();
    ofs.close();
}

InstanceRef InstanceRef::pb_load(const std::string& filename) {
    std::ifstream ifs(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(ifs)),
                      std::istreambuf_iterator<char>());
    ifs.close();
    return pb_loads(data);
}

std::ostream& operator<<(std::ostream& os, const InstanceRef& ref) {
    return os << ref.str();
}

}  // namespace session_cpp
