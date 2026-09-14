#include "instance_ref.h"
#include "tolerance.h"
#include "instance_ref.pb.h"
#include <fstream>
#include <stdexcept>

namespace session_cpp {

InstanceRef::InstanceRef(const InstanceRef &other)
    : name(other.name), definition_guid(other.definition_guid), xform(other.xform), color(other.color), flags(other.flags) {}

InstanceRef &InstanceRef::operator=(const InstanceRef &other) {
  if (this == &other)
    return *this;
  _guid.clear();
  name = other.name;
  definition_guid = other.definition_guid;
  xform = other.xform;
  color = other.color;
  flags = other.flags;
  return *this;
}

// ═══════════════════════════════════════════════════════════════════════════
// Static constructors
// ═══════════════════════════════════════════════════════════════════════════

InstanceRef InstanceRef::with_name(const std::string &name, const std::string &definition_guid, const Xform &xform) {
  InstanceRef ref(definition_guid, xform);
  ref.name = name;
  return ref;
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════

double &InstanceRef::operator[](int index) {
  if (index < 0 || index >= 16)
    throw std::out_of_range("Index out of bounds");
  return xform.m[index];
}

const double &InstanceRef::operator[](int index) const {
  if (index < 0 || index >= 16)
    throw std::out_of_range("Index out of bounds");
  return xform.m[index];
}

bool InstanceRef::operator==(const InstanceRef &other) const {
  return definition_guid == other.definition_guid && xform == other.xform && color == other.color && flags == other.flags;
}

bool InstanceRef::operator!=(const InstanceRef &other) const { return !(*this == other); }

// ═══════════════════════════════════════════════════════════════════════════
// Transformation
// ═══════════════════════════════════════════════════════════════════════════

void InstanceRef::transform(const Xform &t) { xform = t * xform; }

InstanceRef InstanceRef::transformed(const Xform &t) const {
  InstanceRef result = *this;
  result.transform(t);
  return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════

nlohmann::ordered_json InstanceRef::jsondump() const {
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

InstanceRef InstanceRef::jsonload(const nlohmann::json &data) {
  InstanceRef ref(data["definition_guid"], Xform::jsonload(data["xform"]));
  ref.color = Color::jsonload(data["color"]);
  ref.flags = data["flags"];
  ref.guid() = data["guid"];
  ref.name = data["name"];
  return ref;
}

std::string InstanceRef::file_json_dumps() const { return jsondump().dump(); }

InstanceRef InstanceRef::file_json_loads(const std::string &json_string) { return jsonload(nlohmann::ordered_json::parse(json_string)); }

void InstanceRef::file_json_dump(const std::string &filename) const {
  std::ofstream file(filename);
  file << jsondump().dump(4);
}

InstanceRef InstanceRef::file_json_load(const std::string &filename) {
  std::ifstream file(filename);
  return jsonload(nlohmann::json::parse(file));
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════

std::string InstanceRef::pb_dumps() const {
  session_proto::InstanceRef proto;
  if (has_guid())
    proto.set_guid(guid());
  proto.set_name(name);
  proto.set_definition_guid(definition_guid);
  session_proto::Xform *proto_xform = proto.mutable_xform();
  proto_xform->set_name(xform.name);
  for (int i = 0; i < 16; i++)
    proto_xform->add_matrix(xform.m[i]);
  session_proto::Color *proto_color = proto.mutable_color();
  proto_color->set_name(color.name);
  proto_color->set_r(color.r);
  proto_color->set_g(color.g);
  proto_color->set_b(color.b);
  proto_color->set_a(color.a);
  proto.set_flags(flags);
  return proto.SerializeAsString();
}

InstanceRef InstanceRef::pb_loads(const std::string &data) {
  session_proto::InstanceRef proto;
  proto.ParseFromString(data);
  InstanceRef ref;
  if (!proto.guid().empty())
    ref.guid() = proto.guid();
  ref.name = proto.name();
  ref.definition_guid = proto.definition_guid();
  const session_proto::Xform &proto_xform = proto.xform();
  ref.xform.name = proto_xform.name();
  for (int i = 0; i < proto_xform.matrix_size() && i < 16; i++)
    ref.xform.m[i] = proto_xform.matrix(i);
  const session_proto::Color &proto_color = proto.color();
  ref.color.name = proto_color.name();
  ref.color.r = proto_color.r();
  ref.color.g = proto_color.g();
  ref.color.b = proto_color.b();
  ref.color.a = proto_color.a();
  ref.flags = proto.flags();
  return ref;
}

void InstanceRef::pb_dump(const std::string &filename) const {
  const std::string data = pb_dumps();
  std::ofstream file(filename, std::ios::binary);
  file.write(data.data(), data.size());
}

InstanceRef InstanceRef::pb_load(const std::string &filename) {
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

std::ostream &operator<<(std::ostream &os, const InstanceRef &ref) { return os << ref.str(); }

} // namespace session_cpp
