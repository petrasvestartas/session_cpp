#include "color.h"

#include "color.pb.h"
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════

Color::Color(const Color &other)
    : name(other.name), r(other.r), g(other.g), b(other.b), a(other.a) {}

Color &Color::operator=(const Color &other) {

  if (this != &other) {
    name = other.name;
    _guid.clear();
    r = other.r;
    g = other.g;
    b = other.b;
    a = other.a;
  }

  return *this;
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════

float &Color::operator[](int index) {

  if (index == 0)
    return r;

  if (index == 1)
    return g;

  if (index == 2)
    return b;

  if (index == 3)
    return a;

  throw std::out_of_range("Index out of range");
}

const float &Color::operator[](int index) const {

  if (index == 0)
    return r;

  if (index == 1)
    return g;

  if (index == 2)
    return b;

  if (index == 3)
    return a;

  throw std::out_of_range("Index out of range");
}

bool Color::operator==(const Color &other) const {
  return name == other.name && r == other.r && g == other.g && b == other.b &&
         a == other.a;
}

bool Color::operator!=(const Color &other) const { return !(*this == other); }

// ═══════════════════════════════════════════════════════════════════════════
// Presets
// ═══════════════════════════════════════════════════════════════════════════

Color Color::white() { return Color(1.0f, 1.0f, 1.0f, 1.0f, "white"); }
Color Color::black() { return Color(0.0f, 0.0f, 0.0f, 1.0f, "black"); }
Color Color::grey() { return Color(0.5f, 0.5f, 0.5f, 1.0f, "grey"); }
Color Color::red() { return Color(1.0f, 0.0f, 0.0f, 1.0f, "red"); }
Color Color::orange() { return Color(1.0f, 0.5f, 0.0f, 1.0f, "orange"); }
Color Color::yellow() { return Color(1.0f, 1.0f, 0.0f, 1.0f, "yellow"); }
Color Color::lime() { return Color(0.5f, 1.0f, 0.0f, 1.0f, "lime"); }
Color Color::green() { return Color(0.0f, 1.0f, 0.0f, 1.0f, "green"); }
Color Color::mint() { return Color(0.0f, 1.0f, 0.5f, 1.0f, "mint"); }
Color Color::cyan() { return Color(0.0f, 1.0f, 1.0f, 1.0f, "cyan"); }
Color Color::azure() { return Color(0.0f, 0.5f, 1.0f, 1.0f, "azure"); }
Color Color::blue() { return Color(0.0f, 0.0f, 1.0f, 1.0f, "blue"); }
Color Color::violet() { return Color(0.5f, 0.0f, 1.0f, 1.0f, "violet"); }
Color Color::magenta() { return Color(1.0f, 0.0f, 1.0f, 1.0f, "magenta"); }
Color Color::pink() { return Color(1.0f, 0.0f, 0.5f, 1.0f, "pink"); }
Color Color::maroon() { return Color(0.5f, 0.0f, 0.0f, 1.0f, "maroon"); }
Color Color::brown() { return Color(0.5f, 0.25f, 0.0f, 1.0f, "brown"); }
Color Color::olive() { return Color(0.5f, 0.5f, 0.0f, 1.0f, "olive"); }
Color Color::teal() { return Color(0.0f, 0.5f, 0.5f, 1.0f, "teal"); }
Color Color::navy() { return Color(0.0f, 0.0f, 0.5f, 1.0f, "navy"); }
Color Color::purple() { return Color(0.5f, 0.0f, 0.5f, 1.0f, "purple"); }
Color Color::silver() { return Color(0.75f, 0.75f, 0.75f, 1.0f, "silver"); }

std::vector<Color> Color::palette() {
  return {red(),  orange(), yellow(), lime(),   green(),   mint(),
          cyan(), azure(),  blue(),   violet(), magenta(), pink()};
}

// ═══════════════════════════════════════════════════════════════════════════
// Conversion
// ═══════════════════════════════════════════════════════════════════════════

std::array<float, 4> Color::to_unified_array() const { return {r, g, b, a}; }

Color Color::from_unified_array(std::array<float, 4> arr) {
  return Color(arr[0], arr[1], arr[2], arr[3]);
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════

nlohmann::ordered_json Color::jsondump() const {

  nlohmann::ordered_json data;
  data["a"] = a;
  data["b"] = b;
  data["g"] = g;
  data["guid"] = guid();
  data["name"] = name;
  data["r"] = r;
  data["type"] = "Color";

  return data;
}

Color Color::jsonload(const nlohmann::json &data) {

  Color color(data["r"].get<float>(), data["g"].get<float>(),
              data["b"].get<float>(), data["a"].get<float>(), data["name"]);
  color.guid() = data["guid"];

  return color;
}

std::string Color::file_json_dumps() const { return jsondump().dump(); }

Color Color::file_json_loads(const std::string &json_string) {
  return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Color::file_json_dump(const std::string &filename) const {

  std::ofstream file(filename);

  if (!file)
    throw std::runtime_error("Failed to open JSON file: " + filename);

  file << jsondump().dump(4);

  if (!file)
    throw std::runtime_error("Failed to write JSON file: " + filename);
}

Color Color::file_json_load(const std::string &filename) {

  std::ifstream file(filename);

  if (!file)
    throw std::runtime_error("Failed to open JSON file: " + filename);

  return jsonload(nlohmann::json::parse(file));
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════

session_proto::Color Color::to_proto() const {

  session_proto::Color proto;

  if (has_guid())
    proto.set_guid(guid());

  proto.set_name(name);
  proto.set_r(r);
  proto.set_g(g);
  proto.set_b(b);
  proto.set_a(a);

  return proto;
}

Color Color::from_proto(const session_proto::Color &proto) {

  Color color(proto.r(), proto.g(), proto.b(), proto.a(), proto.name());

  if (!proto.guid().empty())
    color.guid() = proto.guid();

  return color;
}

std::string Color::pb_dumps() const { return to_proto().SerializeAsString(); }

Color Color::pb_loads(const std::string &data) {

  session_proto::Color proto;

  if (!proto.ParseFromString(data))
    throw std::runtime_error("Failed to parse Color protobuf data");

  return from_proto(proto);
}

void Color::pb_dump(const std::string &filename) const {

  const std::string data = pb_dumps();
  std::ofstream file(filename, std::ios::binary);

  if (!file)
    throw std::runtime_error("Failed to open protobuf file: " + filename);

  file.write(data.data(), data.size());

  if (!file)
    throw std::runtime_error("Failed to write protobuf file: " + filename);
}

Color Color::pb_load(const std::string &filename) {

  std::ifstream file(filename, std::ios::binary);

  if (!file)
    throw std::runtime_error("Failed to open protobuf file: " + filename);

  const std::string data((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());

  if (file.bad())
    throw std::runtime_error("Failed to read protobuf file: " + filename);

  return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════

std::string Color::str() const {
  return fmt::format("{:.1f}, {:.1f}, {:.1f}, {:.1f}", r, g, b, a);
}

std::string Color::repr() const {
  return fmt::format("Color({}, {:.1f}, {:.1f}, {:.1f}, {:.1f})", name, r, g, b,
                     a);
}

std::ostream &operator<<(std::ostream &os, const Color &color) {
  return os << color.str();
}

} // namespace session_cpp
