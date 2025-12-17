#include "color.h"

#ifdef ENABLE_PROTOBUF
#include "color.pb.h"
#endif

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// Copy Constructor and Assignment
///////////////////////////////////////////////////////////////////////////////////////////

/// Copy constructor (creates a new guid while copying data)
Color::Color(const Color &other)
    : name(other.name),
      guid(::guid()),
      r(other.r),
      g(other.g),
      b(other.b),
      a(other.a) {}

/// Copy assignment (creates a new guid while copying data)
Color &Color::operator=(const Color &other) {
  if (this != &other) {
    name = other.name;
    guid = ::guid();
    r = other.r;
    g = other.g;
    b = other.b;
    a = other.a;
  }
  return *this;
}

///////////////////////////////////////////////////////////////////////////////////////////
// No-copy Operators (index access)
///////////////////////////////////////////////////////////////////////////////////////////

unsigned int &Color::operator[](int index) {
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

const unsigned int &Color::operator[](int index) const {
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

///////////////////////////////////////////////////////////////////////////////////////////
// JSON
///////////////////////////////////////////////////////////////////////////////////////////

nlohmann::ordered_json Color::jsondump() const {
  // Alphabetical order to match Rust's serde_json
  nlohmann::ordered_json data;
  data["a"] = static_cast<int>(a);
  data["b"] = static_cast<int>(b);
  data["g"] = static_cast<int>(g);
  data["guid"] = guid;
  data["name"] = name;
  data["r"] = static_cast<int>(r);
  data["type"] = "Color";
  return data;
}

Color Color::jsonload(const nlohmann::json &data) {
  Color color(static_cast<unsigned int>(data["r"]),
                      static_cast<unsigned int>(data["g"]),
                      static_cast<unsigned int>(data["b"]),
                      static_cast<unsigned int>(data["a"]), data["name"]);
  color.guid = data["guid"];
  return color;
}

void Color::json_dump(const std::string& filename) const {
  std::ofstream file(filename);
  file << jsondump().dump(4);
}

Color Color::json_load(const std::string& filename) {
  std::ifstream file(filename);
  nlohmann::json data = nlohmann::json::parse(file);
  return jsonload(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Protobuf
///////////////////////////////////////////////////////////////////////////////////////////

#ifdef ENABLE_PROTOBUF
std::string Color::to_protobuf() const {
  session_proto::Color proto;
  proto.set_guid(guid);
  proto.set_name(name);
  proto.set_r(r);
  proto.set_g(g);
  proto.set_b(b);
  proto.set_a(a);
  return proto.SerializeAsString();
}

Color Color::from_protobuf(const std::string& data) {
  session_proto::Color proto;
  proto.ParseFromString(data);
  
  Color color(proto.r(), proto.g(), proto.b(), proto.a(), proto.name());
  color.guid = proto.guid();
  return color;
}

void Color::protobuf_dump(const std::string& filename) const {
  std::string data = to_protobuf();
  std::ofstream file(filename, std::ios::binary);
  file.write(data.data(), data.size());
}

Color Color::protobuf_load(const std::string& filename) {
  std::ifstream file(filename, std::ios::binary);
  std::string data((std::istreambuf_iterator<char>(file)),
                    std::istreambuf_iterator<char>());
  return from_protobuf(data);
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////
// Operators
///////////////////////////////////////////////////////////////////////////////////////////

/// Simple string representation (like Python __str__): "r, g, b, a"
std::string Color::str() const {
  return fmt::format("{}, {}, {}, {}", r, g, b, a);
}

/// Detailed representation (like Python __repr__): "Color(name, r, g, b, a)"
std::string Color::repr() const {
  return fmt::format("Color({}, {}, {}, {}, {})", name, r, g, b, a);
}

/// Alias for repr() - for compatibility
std::string Color::to_string() const {
  return repr();
}

/// Equality operator
bool Color::operator==(const Color &other) const {
  return r == other.r && g == other.g && b == other.b && a == other.a &&
         name == other.name;
}

/// Inequality operator
bool Color::operator!=(const Color &other) const { return !(*this == other); }

///////////////////////////////////////////////////////////////////////////////////////////
// Details
///////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////
// Presets
///////////////////////////////////////////////////////////////////////////////////////////

Color Color::white() { return Color(255, 255, 255, 255, "white"); }

Color Color::black() { return Color(0, 0, 0, 255, "black"); }

Color Color::grey() { return Color(128, 128, 128, 255, "grey"); }

Color Color::red() { return Color(255, 0, 0, 255, "red"); }

Color Color::orange() { return Color(255, 128, 0, 255, "orange"); }

Color Color::yellow() { return Color(255, 255, 0, 255, "yellow"); }

Color Color::lime() { return Color(128, 255, 0, 255, "lime"); }

Color Color::green() { return Color(0, 255, 0, 255, "green"); }

Color Color::mint() { return Color(0, 255, 128, 255, "mint"); }

Color Color::cyan() { return Color(0, 255, 255, 255, "cyan"); }

Color Color::azure() { return Color(0, 128, 255, 255, "azure"); }

Color Color::blue() { return Color(0, 0, 255, 255, "blue"); }

Color Color::violet() { return Color(128, 0, 255, 255, "violet"); }

Color Color::magenta() { return Color(255, 0, 255, 255, "magenta"); }

Color Color::pink() { return Color(255, 0, 128, 255, "pink"); }

Color Color::maroon() { return Color(128, 0, 0, 255, "maroon"); }

Color Color::brown() { return Color(128, 64, 0, 255, "brown"); }

Color Color::olive() { return Color(128, 128, 0, 255, "olive"); }

Color Color::teal() { return Color(0, 128, 128, 255, "teal"); }

Color Color::navy() { return Color(0, 0, 128, 255, "navy"); }

Color Color::purple() { return Color(128, 0, 128, 255, "purple"); }

Color Color::silver() { return Color(192, 192, 192, 255, "silver"); }

///////////////////////////////////////////////////////////////////////////////////////////
// Details
///////////////////////////////////////////////////////////////////////////////////////////

std::array<double, 4> Color::to_unified_array() const {
  return {r / 255.0, g / 255.0, b / 255.0, a / 255.0};
}

Color Color::from_unified_array(std::array<double, 4> arr) {
  return Color(static_cast<unsigned int>(arr[0] * 255.0 + 0.5),
               static_cast<unsigned int>(arr[1] * 255.0 + 0.5),
               static_cast<unsigned int>(arr[2] * 255.0 + 0.5),
               static_cast<unsigned int>(arr[3] * 255.0 + 0.5));
}

///////////////////////////////////////////////////////////////////////////////////////////
// Not class methods
///////////////////////////////////////////////////////////////////////////////////////////

std::ostream &operator<<(std::ostream &os, const Color &color) {
  return os << color.to_string();
}

} // namespace session_cpp
