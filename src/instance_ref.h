#pragma once
#include "color.h"
#include "guid.h"
#include "json.h"
#include "xform.h"
#include "fmt/core.h"
#include <cstdint>
#include <ostream>
#include <string>

namespace session_cpp {

/// A block reference: places a definition (by guid) at a transform
class InstanceRef {
public:
  std::string name = "my_instance_ref";
  std::string definition_guid;
  Xform xform;
  Color color = Color::white();
  uint32_t flags = 0;

  InstanceRef() {}

  InstanceRef(const std::string &definition_guid, const Xform &xform)
      : definition_guid(definition_guid), xform(xform) {}

  /// Copy constructor (new guid, same data)
  InstanceRef(const InstanceRef &other);

  /// Copy assignment (new guid, same data)
  InstanceRef &operator=(const InstanceRef &other);

  /// Move keeps the guid; declaring it stops `return x;` from falling back to the guid-minting copy
  InstanceRef(InstanceRef &&other) noexcept = default;
  InstanceRef &operator=(InstanceRef &&other) noexcept = default;

  bool has_guid() const { return !_guid.empty(); }
  const std::string &guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
  std::string &guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

  // ═══════════════════════════════════════════════════════════════════════════
  // Static constructors
  // ═══════════════════════════════════════════════════════════════════════════

  /// Instance with a name, a definition guid and a placement
  static InstanceRef with_name(const std::string &name, const std::string &definition_guid, const Xform &xform);

  // ═══════════════════════════════════════════════════════════════════════════
  // Operators
  // ═══════════════════════════════════════════════════════════════════════════

  /// Placement matrix entry by index (0..15, column-major)
  double &operator[](int index);
  const double &operator[](int index) const;

  bool operator==(const InstanceRef &other) const;
  bool operator!=(const InstanceRef &other) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Transformation
  // ═══════════════════════════════════════════════════════════════════════════

  /// Compose in place: xform = t * xform
  void transform(const Xform &t);

  /// Composed copy
  InstanceRef transformed(const Xform &t) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // JSON
  // ═══════════════════════════════════════════════════════════════════════════

  nlohmann::ordered_json jsondump() const;
  static InstanceRef jsonload(const nlohmann::json &data);
  std::string file_json_dumps() const;
  static InstanceRef file_json_loads(const std::string &json_string);
  void file_json_dump(const std::string &filename) const;
  static InstanceRef file_json_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // Protobuf
  // ═══════════════════════════════════════════════════════════════════════════

  std::string pb_dumps() const;
  static InstanceRef pb_loads(const std::string &data);
  void pb_dump(const std::string &filename) const;
  static InstanceRef pb_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // String
  // ═══════════════════════════════════════════════════════════════════════════

  /// "definition_guid @ [tx, ty, tz]"
  std::string str() const;

  /// "InstanceRef(name, definition_guid, Color(...), flags)"
  std::string repr() const;

private:
  mutable std::string _guid;
};

std::ostream &operator<<(std::ostream &os, const InstanceRef &ref);

} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::InstanceRef> {
  constexpr auto parse(fmt::format_parse_context &ctx) { return ctx.begin(); }

  auto format(const session_cpp::InstanceRef &ref, fmt::format_context &ctx) const {
    return fmt::format_to(ctx.out(), "{}", ref.str());
  }
};
