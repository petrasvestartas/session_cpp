#include "color.h"

#include "color.pb.h"

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// Copy Constructor and Assignment
///////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////
// No-copy Operators (index access)
///////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////
// JSON
///////////////////////////////////////////////////////////////////////////////////////////

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
  Color color(data["r"].get<float>(),
              data["g"].get<float>(),
              data["b"].get<float>(),
              data["a"].get<float>(), data["name"]);
  color.guid() = data["guid"];
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

///////////////////////////////////////////////////////////////////////////////////////////
// Protobuf
///////////////////////////////////////////////////////////////////////////////////////////

std::string Color::pb_dumps() const {
  session_proto::Color proto;
  proto.set_guid(guid());
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
  color.guid() = proto.guid();
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

///////////////////////////////////////////////////////////////////////////////////////////
// Operators
///////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////
// Details
///////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////
// Presets
///////////////////////////////////////////////////////////////////////////////////////////

Color Color::white()   { thread_local static Color _c(1.0f, 1.0f, 1.0f, 1.0f, "white");     return _c; }
Color Color::black()   { thread_local static Color _c(0.0f, 0.0f, 0.0f, 1.0f, "black");     return _c; }
Color Color::grey()    { thread_local static Color _c(0.5f, 0.5f, 0.5f, 1.0f, "grey");      return _c; }
Color Color::red()     { thread_local static Color _c(1.0f, 0.0f, 0.0f, 1.0f, "red");       return _c; }
Color Color::orange()  { thread_local static Color _c(1.0f, 0.5f, 0.0f, 1.0f, "orange");    return _c; }
Color Color::yellow()  { thread_local static Color _c(1.0f, 1.0f, 0.0f, 1.0f, "yellow");    return _c; }
Color Color::lime()    { thread_local static Color _c(0.5f, 1.0f, 0.0f, 1.0f, "lime");      return _c; }
Color Color::green()   { thread_local static Color _c(0.0f, 1.0f, 0.0f, 1.0f, "green");     return _c; }
Color Color::mint()    { thread_local static Color _c(0.0f, 1.0f, 0.5f, 1.0f, "mint");      return _c; }
Color Color::cyan()    { thread_local static Color _c(0.0f, 1.0f, 1.0f, 1.0f, "cyan");      return _c; }
Color Color::azure()   { thread_local static Color _c(0.0f, 0.5f, 1.0f, 1.0f, "azure");     return _c; }
Color Color::blue()    { thread_local static Color _c(0.0f, 0.0f, 1.0f, 1.0f, "blue");      return _c; }
Color Color::violet()  { thread_local static Color _c(0.5f, 0.0f, 1.0f, 1.0f, "violet");    return _c; }
Color Color::magenta() { thread_local static Color _c(1.0f, 0.0f, 1.0f, 1.0f, "magenta");   return _c; }
Color Color::pink()    { thread_local static Color _c(1.0f, 0.0f, 0.5f, 1.0f, "pink");      return _c; }
Color Color::maroon()  { thread_local static Color _c(0.5f, 0.0f, 0.0f, 1.0f, "maroon");    return _c; }
Color Color::brown()   { thread_local static Color _c(0.5f, 0.25f, 0.0f, 1.0f, "brown");    return _c; }
Color Color::olive()   { thread_local static Color _c(0.5f, 0.5f, 0.0f, 1.0f, "olive");     return _c; }
Color Color::teal()    { thread_local static Color _c(0.0f, 0.5f, 0.5f, 1.0f, "teal");      return _c; }
Color Color::navy()    { thread_local static Color _c(0.0f, 0.0f, 0.5f, 1.0f, "navy");      return _c; }
Color Color::purple()  { thread_local static Color _c(0.5f, 0.0f, 0.5f, 1.0f, "purple");    return _c; }
Color Color::silver()  { thread_local static Color _c(0.75f, 0.75f, 0.75f, 1.0f, "silver"); return _c; }

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

///////////////////////////////////////////////////////////////////////////////////////////
// Details
///////////////////////////////////////////////////////////////////////////////////////////

std::array<float, 4> Color::to_unified_array() const {
  return {r, g, b, a};
}

Color Color::from_unified_array(std::array<float, 4> arr) {
  return Color(arr[0], arr[1], arr[2], arr[3]);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Not class methods
///////////////////////////////////////////////////////////////////////////////////////////

std::ostream &operator<<(std::ostream &os, const Color &color) {
  return os << color.str();
}

} // namespace session_cpp
