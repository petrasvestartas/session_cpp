#pragma once
#include "fmt/core.h"
#include "guid.h"
#include "json.h"
#include <array>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace session_cpp {

/**
 * @class Color
 * @brief A color is defined by RGBA coordinates from 0 to 255.
 */
class Color {
public:
  std::string name = "my_color"; ///< Name of the color
  std::string guid = ::guid();   ///< Unique identifier
  unsigned int r;                ///< Red component (0-255)
  unsigned int g;                ///< Green component (0-255)
  unsigned int b;                ///< Blue component (0-255)
  unsigned int a;                ///< Alpha component (0-255)

  /**
   * @brief Constructor with RGBA values.
   * @param r Red component (0-255). Default: 255.
   * @param g Green component (0-255). Default: 255.
   * @param b Blue component (0-255). Default: 255.
   * @param a Alpha component (0-255). Default: 255.
   * @param name Name for the color. Default: "my_color".
   */
  Color(unsigned int r = 255, unsigned int g = 255, unsigned int b = 255,
        unsigned int a = 255, std::string name = "my_color")
      : name(name), r(r), g(g), b(b), a(a) {}

  /// Copy constructor (creates a new guid while copying data)
  Color(const Color &other);

  /// Copy assignment (creates a new guid while copying data)
  Color &operator=(const Color &other);

  /**
   * @brief Create a copy of this color with a new GUID.
   * @return A new Color with identical RGBA values but a different GUID.
   */
  Color duplicate() const;

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Operators
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Simple string representation (like Python __str__): "r, g, b, a"
  std::string str() const;

  /// Detailed representation (like Python __repr__): "Color(name, r, g, b, a)"
  std::string repr() const;

  /// Alias for repr() - for compatibility
  std::string to_string() const;

  /// Equality operator
  bool operator==(const Color &other) const;

  /// Inequality operator
  bool operator!=(const Color &other) const;

  ///////////////////////////////////////////////////////////////////////////////////////////
  // No-copy Operators (index access)
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Get/set color component by index (0=r, 1=g, 2=b, 3=a)
  unsigned int &operator[](int index);

  /// Get color component by index (const version)
  const unsigned int &operator[](int index) const;

  ///////////////////////////////////////////////////////////////////////////////////////////
  // JSON
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Convert to JSON-serializable object.
  nlohmann::ordered_json jsondump() const;

  /// Create color from JSON data.
  static Color jsonload(const nlohmann::json &data);

  /// Write JSON to file
  void json_dump(const std::string& filename) const;

  /// Read JSON from file
  static Color json_load(const std::string& filename);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Protobuf
  ///////////////////////////////////////////////////////////////////////////////////////////

#ifdef ENABLE_PROTOBUF
  /// Convert to protobuf message and serialize to binary
  std::string to_protobuf() const;

  /// Deserialize from protobuf binary
  static Color from_protobuf(const std::string& data);

  /// Write protobuf to file
  void protobuf_dump(const std::string& filename) const;

  /// Read protobuf from file
  static Color protobuf_load(const std::string& filename);
#endif

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Details
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Create a white color.
  static Color white();

  /// Create a black color.
  static Color black();

  /// Create a grey color.
  static Color grey();

  /// Create a red color.
  static Color red();

  /// Create an orange color.
  static Color orange();

  /// Create a yellow color.
  static Color yellow();

  /// Create a lime color.
  static Color lime();

  /// Create a green color.
  static Color green();

  /// Create a mint color.
  static Color mint();

  /// Create a cyan color.
  static Color cyan();

  /// Create an azure color.
  static Color azure();

  /// Create a blue color.
  static Color blue();

  /// Create a violet color.
  static Color violet();

  /// Create a magenta color.
  static Color magenta();

  /// Create a pink color.
  static Color pink();

  /// Create a maroon color.
  static Color maroon();

  /// Create a brown color.
  static Color brown();

  /// Create an olive color.
  static Color olive();

  /// Create a teal color.
  static Color teal();

  /// Create a navy color.
  static Color navy();

  /// Create a purple color.
  static Color purple();

  /// Create a silver color.
  static Color silver();

  /**
   * @brief Convert to normalized float array [0-1].
   * @return Array [r, g, b, a] with values normalized to [0.0, 1.0].
   */
  std::array<double, 4> to_unified_array() const;

  /**
   * @brief Create color from normalized float values [0-1].
   * @param arr Array [r, g, b, a] with values in [0.0, 1.0] range.
   * @return A new Color with values converted to 0-255 range.
   */
  static Color from_unified_array(std::array<double, 4> arr);
};

/**
 * @brief  To use this operator, you can do:
 *         Point point(1.5, 2.5, 3.5);
 *         std::cout << "Created point: " << point << std::endl;
 * @param os The output stream.
 * @param point The Point to insert into the stream.
 * @return A reference to the output stream.
 */
std::ostream &operator<<(std::ostream &os, const Color &color);

} // namespace session_cpp

// fmt formatter specialization for Color - enables direct fmt::print(color)
template <> struct fmt::formatter<session_cpp::Color> {
  constexpr auto parse(fmt::format_parse_context &ctx) { return ctx.begin(); }

  auto format(const session_cpp::Color &o, fmt::format_context &ctx) const {
    return fmt::format_to(ctx.out(), "{}", o.to_string());
  }
};
