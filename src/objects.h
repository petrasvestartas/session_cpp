#pragma once
#include "guid.h"  // For ::guid()
#include "json.h"  // For nlohmann::ordered_json
#include "point.h"
#include "vector.h"
#include "line.h"
#include "plane.h"
#include "obb.h"
#include "polyline.h"
#include "pointcloud.h"
#include "mesh.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "brep.h"
#include "element.h"
#include <fstream>
#include <memory>
#include <string>
#include <variant>
#include <vector>

namespace session_cpp {

/**
 * @struct Component
 * @brief A custom domain object stored generically in a Session.
 *
 * Allows external packages (e.g. session_tf) to store arbitrary serializable
 * objects (FloorBuilder, WallBuilder, …) without the core session needing to
 * know their concrete types.  The full JSON dict is preserved in `extra` so
 * downstream code can reconstruct the concrete object via a factory registry.
 */
struct Component {
  std::string type_name;               ///< Class name, e.g. "FloorBuilder"
  std::string name = "my_component";   ///< Human-readable name
  nlohmann::ordered_json extra;        ///< All custom fields (everything except type/guid/name)

  bool has_guid() const { return !_guid.empty(); }
  const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
  std::string& guid()       { if (_guid.empty()) _guid = ::guid(); return _guid; }

  nlohmann::ordered_json jsondump() const {
    nlohmann::ordered_json j = extra;
    j["type"] = type_name;
    j["guid"] = guid();
    j["name"] = name;
    return j;
  }

  static Component jsonload(const nlohmann::json& j) {
    Component c;
    c.type_name   = j.value("type", "");
    c.guid()      = j.value("guid", ::guid());
    c.name        = j.value("name", "my_component");
    c.extra       = j;
    c.extra.erase("type");
    c.extra.erase("guid");
    c.extra.erase("name");
    return c;
  }

  std::string pb_dumps() const;
  static Component pb_loads(const std::string& data);

private:
  mutable std::string _guid;
};

/**
 * @class Objects
 * @brief A collection of geometry objects.
 */
class Objects {
public:
  std::string name = "my_objects"; ///< The name of the objects
  bool has_guid() const { return !_guid.empty(); }
  const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
  std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
  
  // Collections for all geometry types
  std::shared_ptr<std::vector<std::shared_ptr<Point>>> points;
  std::shared_ptr<std::vector<std::shared_ptr<Line>>> lines;
  std::shared_ptr<std::vector<std::shared_ptr<Plane>>> planes;
  std::shared_ptr<std::vector<std::shared_ptr<OBB>>> bboxes;
  std::shared_ptr<std::vector<std::shared_ptr<Polyline>>> polylines;
  std::shared_ptr<std::vector<std::shared_ptr<PointCloud>>> pointclouds;
  std::shared_ptr<std::vector<std::shared_ptr<Mesh>>> meshes;
  std::shared_ptr<std::vector<std::shared_ptr<NurbsCurve>>> nurbscurves;
  std::shared_ptr<std::vector<std::shared_ptr<NurbsSurface>>> nurbssurfaces;
  std::shared_ptr<std::vector<std::shared_ptr<BRep>>> breps;
  std::shared_ptr<std::vector<std::shared_ptr<Element>>> elements;
  std::shared_ptr<std::vector<Component>> components;

  /**
   * @brief Constructor.
   * @param name The name of the collection.
   */
  Objects(std::string name = "my_objects") : name(std::move(name)) {
    // Initialize all geometry collections
    this->points = std::make_shared<std::vector<std::shared_ptr<Point>>>();
    this->lines = std::make_shared<std::vector<std::shared_ptr<Line>>>();
    this->planes = std::make_shared<std::vector<std::shared_ptr<Plane>>>();
    this->bboxes = std::make_shared<std::vector<std::shared_ptr<OBB>>>();
    this->polylines = std::make_shared<std::vector<std::shared_ptr<Polyline>>>();
    this->pointclouds = std::make_shared<std::vector<std::shared_ptr<PointCloud>>>();
    this->meshes = std::make_shared<std::vector<std::shared_ptr<Mesh>>>();
    this->nurbscurves = std::make_shared<std::vector<std::shared_ptr<NurbsCurve>>>();
    this->nurbssurfaces = std::make_shared<std::vector<std::shared_ptr<NurbsSurface>>>();
    this->breps = std::make_shared<std::vector<std::shared_ptr<BRep>>>();
    this->elements   = std::make_shared<std::vector<std::shared_ptr<Element>>>();
    this->components = std::make_shared<std::vector<Component>>();
  }

  /// A copy duplicates every list AND every object in it, identities included.
  ///
  /// The lists are held by shared_ptr, so the implicit copy shared them: two Objects, one set
  /// of geometry, and clearing either cleared both. Guids are put back after each object's own
  /// copy constructor mints a fresh one, because a Session's tree, graph and xforms all key on
  /// them - a duplicate that renamed everything would be a session whose own indexes no longer
  /// matched its geometry.
  Objects(const Objects& other);
  Objects& operator=(const Objects& other);
  /// A move is the same collection in a new place, so the lists transfer as they are.
  Objects(Objects&&) noexcept = default;
  Objects& operator=(Objects&&) noexcept = default;

  /// Convert objects to string representation
  std::string str() const;

  /**
   * @brief Serializes the Objects instance to JSON.
   * @return JSON representation of the Objects instance.
   */
  nlohmann::ordered_json jsondump() const;

  /**
   * @brief Creates an Objects instance from JSON data.
   * @param data JSON data containing objects information.
   * @return Objects instance created from the data.
   */
  static Objects jsonload(const nlohmann::json &data);

  std::string file_json_dumps() const;
  static Objects file_json_loads(const std::string& json_string);
  void file_json_dump(const std::string& filename) const;
  static Objects file_json_load(const std::string& filename);
  std::string pb_dumps() const;
  static Objects pb_loads(const std::string& data);
  void pb_dump(const std::string& filename) const;
  static Objects pb_load(const std::string& filename);

private:
  mutable std::string _guid;
};

/// All geometry types as a variant. A new type joins here and in the checklist at the top of
/// session.cpp.
using Geometry = std::variant<
    std::shared_ptr<OBB>,
    std::shared_ptr<Line>,
    std::shared_ptr<Mesh>,
    std::shared_ptr<Plane>,
    std::shared_ptr<Point>,
    std::shared_ptr<PointCloud>,
    std::shared_ptr<NurbsCurve>,
    std::shared_ptr<NurbsSurface>,
    std::shared_ptr<Polyline>,
    std::shared_ptr<BRep>,
    std::shared_ptr<Element>
>;

/// Anything an Objects collection holds: geometry, or a Component.
using Item = std::variant<Geometry, Component>;

/**
 * @brief  To use this operator, you can do:
 *         Point point(1.5, 2.5, 3.5);
 *         std::cout << "Created point: " << point << std::endl;
 * @param os The output stream.
 * @param point The Point to insert into the stream.
 * @return A reference to the output stream.
 */
std::ostream &operator<<(std::ostream &os, const Objects &objects);
} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::Objects> {
  constexpr auto parse(fmt::format_parse_context &ctx) { return ctx.begin(); }

  auto format(const session_cpp::Objects &o, fmt::format_context &ctx) const {
    return fmt::format_to(ctx.out(), "{}", o.str());
  }
};
