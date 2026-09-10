#include "color.h"

#include "color.pb.h"

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Copy Constructor and Assignment
// ═══════════════════════════════════════════════════════════════════════════

/// Copy constructor (creates a new guid() while copying data)
Color::Color(const Color &other)
    : name(other.name),
      r(other.r),
      g(other.g),
      b(other.b),
      a(other.a) {}

/// Copy assignment (creates a new guid() while copying data)
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
// No-copy Operators (index access)
// ═══════════════════════════════════════════════════════════════════════════

float &Color::operator[](int index) {
  if (index == 0) {
    return r;
  } else if (index == 1) {
    return g;
  } else if (index == 2) {
    return b;
  } else if (index == 3) {
    return a;
  } else {
    throw std::out_of_range("Index out of range");
  }
}

const float &Color::operator[](int index) const {
  if (index == 0) {
    return r;
  } else if (index == 1) {
    return g;
  } else if (index == 2) {
    return b;
  } else if (index == 3) {
    return a;
  } else {
    throw std::out_of_range("Index out of range");
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════

nlohmann::ordered_json Color::jsondump() const {
  // Alphabetical order to match Rust's serde_json
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
  // Indexing a const json with an absent key is undefined; rgb are required, the rest default.
  Color color(data.at("r").get<float>(),
              data.at("g").get<float>(),
              data.at("b").get<float>(),
              data.value("a", 1.0f), data.value("name", std::string("my_color")));
  if (data.contains("guid")) { color.guid() = data["guid"]; }
  return color;
}

std::string Color::file_json_dumps() const {
    return jsondump().dump();
}

Color Color::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Color::file_json_dump(const std::string& filename) const {
  std::ofstream file(filename);
  file << jsondump().dump(4);
}

Color Color::file_json_load(const std::string& filename) {
  std::ifstream file(filename);
  nlohmann::json data = nlohmann::json::parse(file);
  return jsonload(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════

std::string Color::pb_dumps() const {
  session_proto::Color proto;
  if (has_guid()) { proto.set_guid(guid()); }
  proto.set_name(name);
  proto.set_r(r);
  proto.set_g(g);
  proto.set_b(b);
  proto.set_a(a);
  return proto.SerializeAsString();
}

Color Color::pb_loads(const std::string& data) {
  session_proto::Color proto;
  proto.ParseFromString(data);

  Color color(proto.r(), proto.g(), proto.b(), proto.a(), proto.name());
  if (!proto.guid().empty()) { color.guid() = proto.guid(); }
  return color;
}

void Color::pb_dump(const std::string& filename) const {
  std::string data = pb_dumps();
  std::ofstream file(filename, std::ios::binary);
  file.write(data.data(), data.size());
}

Color Color::pb_load(const std::string& filename) {
  std::ifstream file(filename, std::ios::binary);
  std::string data((std::istreambuf_iterator<char>(file)),
                    std::istreambuf_iterator<char>());
  return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════

/// Simple string representation (like Python __str__): "r, g, b, a"
std::string Color::str() const {
  return fmt::format("{:.1f}, {:.1f}, {:.1f}, {:.1f}", r, g, b, a);
}

/// Detailed representation (like Python __repr__): "Color(name, r, g, b, a)"
std::string Color::repr() const {
  return fmt::format("Color({}, {:.1f}, {:.1f}, {:.1f}, {:.1f})", name, r, g, b, a);
}


/// Equality operator
bool Color::operator==(const Color &other) const {
  return r == other.r && g == other.g && b == other.b && a == other.a &&
         name == other.name;
}

/// Inequality operator
bool Color::operator!=(const Color &other) const { return !(*this == other); }

// ═══════════════════════════════════════════════════════════════════════════
// Presets
// ═══════════════════════════════════════════════════════════════════════════

Color Color::white()   { return Color(1.0f, 1.0f, 1.0f, 1.0f, "white"); }
Color Color::black()   { return Color(0.0f, 0.0f, 0.0f, 1.0f, "black"); }
Color Color::grey()    { return Color(0.5f, 0.5f, 0.5f, 1.0f, "grey"); }
Color Color::red()     { return Color(1.0f, 0.0f, 0.0f, 1.0f, "red"); }
Color Color::orange()  { return Color(1.0f, 0.5f, 0.0f, 1.0f, "orange"); }
Color Color::yellow()  { return Color(1.0f, 1.0f, 0.0f, 1.0f, "yellow"); }
Color Color::lime()    { return Color(0.5f, 1.0f, 0.0f, 1.0f, "lime"); }
Color Color::green()   { return Color(0.0f, 1.0f, 0.0f, 1.0f, "green"); }
Color Color::mint()    { return Color(0.0f, 1.0f, 0.5f, 1.0f, "mint"); }
Color Color::cyan()    { return Color(0.0f, 1.0f, 1.0f, 1.0f, "cyan"); }
Color Color::azure()   { return Color(0.0f, 0.5f, 1.0f, 1.0f, "azure"); }
Color Color::blue()    { return Color(0.0f, 0.0f, 1.0f, 1.0f, "blue"); }
Color Color::violet()  { return Color(0.5f, 0.0f, 1.0f, 1.0f, "violet"); }
Color Color::magenta() { return Color(1.0f, 0.0f, 1.0f, 1.0f, "magenta"); }
Color Color::pink()    { return Color(1.0f, 0.0f, 0.5f, 1.0f, "pink"); }
Color Color::maroon()  { return Color(0.5f, 0.0f, 0.0f, 1.0f, "maroon"); }
Color Color::brown()   { return Color(0.5f, 0.25f, 0.0f, 1.0f, "brown"); }
Color Color::olive()   { return Color(0.5f, 0.5f, 0.0f, 1.0f, "olive"); }
Color Color::teal()    { return Color(0.0f, 0.5f, 0.5f, 1.0f, "teal"); }
Color Color::navy()    { return Color(0.0f, 0.0f, 0.5f, 1.0f, "navy"); }
Color Color::purple()  { return Color(0.5f, 0.0f, 0.5f, 1.0f, "purple"); }
Color Color::silver()  { return Color(0.75f, 0.75f, 0.75f, 1.0f, "silver"); }

std::vector<Color> Color::palette() {
    return {
        red(),
        orange(),
        yellow(),
        lime(),
        green(),
        mint(),
        cyan(),
        azure(),
        blue(),
        violet(),
        magenta(),
        pink(),
    };
}

// ═══════════════════════════════════════════════════════════════════════════
// Details
// ═══════════════════════════════════════════════════════════════════════════

std::array<float, 4> Color::to_unified_array() const {
  return {r, g, b, a};
}

Color Color::from_unified_array(std::array<float, 4> arr) {
  return Color(arr[0], arr[1], arr[2], arr[3]);
}

// ═══════════════════════════════════════════════════════════════════════════
// Not class methods
// ═══════════════════════════════════════════════════════════════════════════

std::ostream &operator<<(std::ostream &os, const Color &color) {
  return os << color.str();
}

} // namespace session_cpp
