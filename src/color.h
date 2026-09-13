#pragma once
#include "fmt/core.h"
#include "guid.h"
#include "json.h"
#include <algorithm>
#include <array>
#include <ostream>
#include <string>
#include <vector>

namespace session_proto {
class Color;
}

namespace session_cpp {

/// A color with RGBA components in [0.0, 1.0]
class Color {
public:
  std::string name = "my_color";
  float r;
  float g;
  float b;
  float a;

  /// Construct from RGBA components, each clamped to [0.0, 1.0]
  Color(float r = 1.0f, float g = 1.0f, float b = 1.0f, float a = 1.0f,
        std::string name = "my_color")
      : name(name), r(std::clamp(r, 0.0f, 1.0f)), g(std::clamp(g, 0.0f, 1.0f)),
        b(std::clamp(b, 0.0f, 1.0f)), a(std::clamp(a, 0.0f, 1.0f)) {}

  /// Copy constructor (new guid, same data)
  Color(const Color &other);

  /// Copy assignment (new guid, same data)
  Color &operator=(const Color &other);

  /// Move keeps the guid; declaring it stops `return x;` from falling back to
  /// the guid-minting copy
  Color(Color &&other) noexcept = default;
  Color &operator=(Color &&other) noexcept = default;

  bool has_guid() const { return !_guid.empty(); }
  const std::string &guid() const {
    if (_guid.empty())
      _guid = ::guid();
    return _guid;
  }
  std::string &guid() {
    if (_guid.empty())
      _guid = ::guid();
    return _guid;
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Operators
  // ═══════════════════════════════════════════════════════════════════════════

  /// Component by index (0=r, 1=g, 2=b, 3=a)
  float &operator[](int index);
  const float &operator[](int index) const;

  bool operator==(const Color &other) const;
  bool operator!=(const Color &other) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Presets
  // ═══════════════════════════════════════════════════════════════════════════

  static Color white();
  static Color black();
  static Color grey();
  static Color red();
  static Color orange();
  static Color yellow();
  static Color lime();
  static Color green();
  static Color mint();
  static Color cyan();
  static Color azure();
  static Color blue();
  static Color violet();
  static Color magenta();
  static Color pink();
  static Color maroon();
  static Color brown();
  static Color olive();
  static Color teal();
  static Color navy();
  static Color purple();
  static Color silver();

  /// The 12 spectral colors in order
  static std::vector<Color> palette();

  // ═══════════════════════════════════════════════════════════════════════════
  // Conversion
  // ═══════════════════════════════════════════════════════════════════════════

  /// Components as [r, g, b, a]
  std::array<float, 4> to_unified_array() const;

  /// Color from [r, g, b, a]
  static Color from_unified_array(std::array<float, 4> arr);

  // ═══════════════════════════════════════════════════════════════════════════
  // JSON
  // ═══════════════════════════════════════════════════════════════════════════

  nlohmann::ordered_json jsondump() const;
  static Color jsonload(const nlohmann::json &data);
  std::string file_json_dumps() const;
  static Color file_json_loads(const std::string &json_string);
  void file_json_dump(const std::string &filename) const;
  static Color file_json_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // Protobuf
  // ═══════════════════════════════════════════════════════════════════════════

  /// Convert to the protobuf message
  session_proto::Color to_proto() const;

  /// Construct from the protobuf message
  static Color from_proto(const session_proto::Color &proto);

  std::string pb_dumps() const;
  static Color pb_loads(const std::string &data);
  void pb_dump(const std::string &filename) const;
  static Color pb_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // String
  // ═══════════════════════════════════════════════════════════════════════════

  /// "r, g, b, a"
  std::string str() const;

  /// "Color(name, r, g, b, a)"
  std::string repr() const;

private:
  mutable std::string _guid;
};

std::ostream &operator<<(std::ostream &os, const Color &color);

} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::Color> {
  constexpr auto parse(fmt::format_parse_context &ctx) { return ctx.begin(); }

  auto format(const session_cpp::Color &color, fmt::format_context &ctx) const {
    return fmt::format_to(ctx.out(), "{}", color.str());
  }
};
