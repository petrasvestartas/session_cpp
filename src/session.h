#pragma once
#include "color.h"
#include "fmt/core.h"
#include "graph.h"
#include "guid.h"
#include "json.h"
#include "objects.h"
#include "point.h"
#include "vector.h"
#include "line.h"
#include "plane.h"
#include "obb.h"
#include "polyline.h"
#include "pointcloud.h"
#include "mesh.h"
#include "mesh_offset.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "brep.h"
#include "tree.h"
#include "history.h"
#include "spatial_bvh.h"
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <variant>
#include <memory>

namespace session_cpp {

/// The Objects lists in order() sequence, each with the prefix of its graph node attribute.
inline const std::vector<std::pair<std::string, std::string>> COLLECTIONS = {
    {"points", "point"},
    {"lines", "line"},
    {"planes", "plane"},
    {"bboxes", "bbox"},
    {"polylines", "polyline"},
    {"pointclouds", "pointcloud"},
    {"meshes", "mesh"},
    {"nurbscurves", "nurbscurve"},
    {"nurbssurfaces", "nurbssurface"},
    {"breps", "brep"},
    {"elements", "element"},
    {"components", "component"},
};

/// A session containing geometry objects.
class Session {
public:
  std::string name = "my_session"; ///< The name of the session
  bool has_guid() const { return !_guid.empty(); }
  const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
  std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
  Objects objects;                 ///< Collection of geometry objects
  std::unordered_map<std::string, Geometry>
      lookup; ///< Fast lookup table for objects by GUID
  Tree tree;  ///< Tree structure for hierarchy
  Graph graph; ///< Graph structure for relationships
  std::unordered_map<std::string, Component>
      component_lookup; ///< Fast lookup for custom components by GUID
  /// Guid -> LOCAL transform, relative to the tree parent. THE only place a transform is
  /// stored: geometry types carry no transformation member. Cumulative placement comes from
  /// world_xform(), which multiplies down the tree. Serialized explicitly by
  /// jsondump/pb_dumps in order() sequence (a map has no deterministic order).
  std::unordered_map<std::string, Xform> xforms;
  /// Undo/redo buffer, in memory only. Ops are recorded ONLY while a transaction is open
  /// (begin ... commit), and every save purges it, as Rhino does - which is why it is mutable:
  /// the const dumps clear it.
  mutable History history;
  SpatialBVH bvh;    ///< Bounding volume hierarchy for collision detection
  
  // SpatialBVH caching for ray casting performance
  SpatialBVH cached_ray_bvh;                           ///< Cached SpatialBVH for ray casting
  std::vector<std::string> cached_guids;        ///< GUID mapping for cached SpatialBVH
  std::vector<OBB> cached_boxes;        ///< Cached AABBs (avoid recomputing)
  bool bvh_cache_dirty = true;                  ///< Flag to rebuild SpatialBVH cache

  /// Constructor.
  Session(std::string name = "my_session")
      : name(std::move(name)), objects(),
        tree(this->name + "_tree"), graph(this->name + "_graph") {
    // Create empty root node with session name
    auto root_node = std::make_shared<TreeNode>(this->name);
    tree.add(root_node);
  }

  /// A copy is an INDEPENDENT session holding the same data: every object, every tree node
  /// and every table is duplicated, and nothing is shared with the original. Identities come
  /// across unchanged, because the tree, the graph and the xforms all key on them - a
  /// duplicate that renamed its geometry would be a session whose own indexes no longer
  /// matched it. The BVH caches are not copied; they are marked dirty and rebuilt on demand,
  /// since their nodes are raw pointers into an arena. History starts empty.
  Session(const Session& other);
  Session& operator=(const Session& other);
  Session(Session&&) noexcept = default;
  Session& operator=(Session&&) noexcept = default;

  /// Convert session to string representation
  std::string str() const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Geometry Management
  // ═══════════════════════════════════════════════════════════════════════════

  /// Get a geometry object by GUID with type safety.
  template <typename T> std::shared_ptr<T> get_object(const std::string &guid) {
    auto it = lookup.find(guid);
    if (it == lookup.end())
      return nullptr;
    auto ptr = std::get_if<std::shared_ptr<T>>(&it->second);
    return ptr ? *ptr : nullptr;
  }

  /// Get a geometry object by GUID (const version).
  template <typename T>
  std::shared_ptr<const T> get_object(const std::string &guid) const {
    auto it = lookup.find(guid);
    if (it == lookup.end())
      return nullptr;
    auto ptr = std::get_if<std::shared_ptr<T>>(&it->second);
    return ptr ? *ptr : nullptr;
  }

  /// Select objects of one type, grouped by the top-level nodes of the tree.
  template <typename T> std::vector<std::vector<T>> select_by_type() const {
    std::vector<std::vector<T>> groups;
    std::shared_ptr<TreeNode> root = tree.root();
    if (!root)
      return groups;
    for (TreeNode *group : root->children()) {
      std::vector<T> items;
      for (TreeNode *node : group->descendants()) {
        if (std::shared_ptr<const T> object = get_object<T>(node->name))
          items.push_back(*object);
      }
      if (!items.empty())
        groups.push_back(std::move(items));
    }
    return groups;
  }

  /// Every add_* below SKIPS an object that is null or carries nothing to draw, and returns

  /// Add a point to the session.
  std::shared_ptr<TreeNode> add_point(std::shared_ptr<Point> point, std::shared_ptr<TreeNode> parent = nullptr);

  /// Add a line to the session.
  std::shared_ptr<TreeNode> add_line(std::shared_ptr<Line> line, std::shared_ptr<TreeNode> parent = nullptr);

  /// Add a plane to the session.
  std::shared_ptr<TreeNode> add_plane(std::shared_ptr<Plane> plane, std::shared_ptr<TreeNode> parent = nullptr);

  /// Add a bounding box to the session.
  std::shared_ptr<TreeNode> add_obb(std::shared_ptr<OBB> bbox);

  /// Add a polyline to the session.
  std::shared_ptr<TreeNode> add_polyline(std::shared_ptr<Polyline> polyline, std::shared_ptr<TreeNode> parent = nullptr);

  /// Add a point cloud to the session.
  std::shared_ptr<TreeNode> add_pointcloud(std::shared_ptr<PointCloud> pointcloud, std::shared_ptr<TreeNode> parent = nullptr);

  /// Add a mesh to the session.
  std::shared_ptr<TreeNode> add_mesh(std::shared_ptr<Mesh> mesh, std::shared_ptr<TreeNode> parent = nullptr);

  /// Add a curve. Null, or fewer than two control vertices, adds nothing and returns nullptr.
  std::shared_ptr<TreeNode> add_nurbscurve(std::shared_ptr<NurbsCurve> nurbscurve, std::shared_ptr<TreeNode> parent = nullptr);

  /// Add a surface. Null, or no control vertices, adds nothing and returns nullptr.
  std::shared_ptr<TreeNode> add_nurbssurface(std::shared_ptr<NurbsSurface> nurbssurface, std::shared_ptr<TreeNode> parent = nullptr);

  /// Add a brep. Null, or no faces and no vertices, adds nothing and returns nullptr.
  std::shared_ptr<TreeNode> add_brep(std::shared_ptr<BRep> brep, std::shared_ptr<TreeNode> parent = nullptr);

  /// Add an element. Only a null element adds nothing: an Element is a data record and is kept
  /// even when it carries no geometry.
  std::shared_ptr<TreeNode> add_element(std::shared_ptr<Element> element, std::shared_ptr<TreeNode> parent = nullptr);

  /// Add a custom component (any object with type_name/guid/name/extra).
  std::shared_ptr<TreeNode> add_component(Component component, std::shared_ptr<TreeNode> parent = nullptr);

  /// Add a TreeNode to the tree hierarchy.
  void add(std::shared_ptr<TreeNode> node,
           std::shared_ptr<TreeNode> parent = nullptr);

  /// Create a named layer (TreeNode) and add it to the root of the tree.
  std::shared_ptr<TreeNode> add_group(const std::string& group_name);

  /// Find an existing group by name. Throws std::runtime_error if not found.
  std::shared_ptr<TreeNode> find_group(const std::string& group_name) const;

  /// Add geometry to session (stores object and adds to tree)
  void add_surface(std::shared_ptr<NurbsSurface> surface);
  void add_curve(std::shared_ptr<NurbsCurve> curve);

  /// Add an edge between two geometry objects in the graph.
  void add_edge(const std::string &guid1, const std::string &guid2,
                const std::string &attribute = "");

  /// Compute face-to-face contacts between all elements.
  /// Uses SpatialBVH + OBB for adjacency, then boolean intersection for contact areas.

  /// Remove an object by its GUID from every live table at once: its typed list, lookup, its
  /// xform, its tree node (with the subtree) and its graph node with every incident edge. The
  /// removal record is the tombstone that undo restores from.
  bool remove_object(const std::string &obj_guid);

  /// Swap the object stored under guid for obj, which takes over that guid, in its typed list
  /// and lookup, and refresh its graph node attribute. This is the recorded edit: undo restores
  /// the previous object, redo the new one, as absolute snapshots. Mutating an object in place
  /// through lookup stays possible and is NOT recorded - history only sees what goes through
  /// replace. Returns false when the guid is not found.
  bool replace(const std::string &guid, const Geometry &obj);

  /// Canonical object order: the objects vectors walked in one fixed type sequence —
  std::vector<std::string> order() const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Xforms - the one place a transformation is stored
  // ═══════════════════════════════════════════════════════════════════════════

  /// Sets the LOCAL transform of an object, relative to its tree parent.
  void set_xform(const std::string &guid, const Xform &xform);

  /// The LOCAL transform of an object, identity when none was set.
  Xform xform(const std::string &guid) const;

  /// Removes an object's local transform, returning whether one was present.
  bool remove_xform(const std::string &guid);

  /// The CUMULATIVE placement of an object: every ancestor's transform multiplied down the
  /// tree onto its own. An object with no tree node is its own root and returns its local
  /// transform - objects added without a parent are never attached, so treating a missing
  /// node as identity would silently move them to the origin.
  Xform world_xform(const std::string &guid) const;

  /// Every object's cumulative placement, computed in ONE downward pass. Use this instead of
  /// calling world_xform() per object: that does a whole-tree scan to find each node, which
  /// is quadratic over a session.
  std::unordered_map<std::string, Xform> world_xforms() const;

  // ═══════════════════════════════════════════════════════════════════════════
  // History
  // ═══════════════════════════════════════════════════════════════════════════

  /// Open a history transaction: every add, remove, replace and xform change until commit
  /// becomes one undo step.
  void begin(const std::string &label);
  void commit();
  bool undo();
  bool redo();

  // ═══════════════════════════════════════════════════════════════════════════
  // Tree Operations
  // ═══════════════════════════════════════════════════════════════════════════

  /// Add a parent-child relationship in the tree.
  bool add_hierarchy(const std::string &parent_guid,
                     const std::string &child_guid);

  /// Get the children of a parent GUID
  std::vector<std::string> get_children(const std::string &obj_guid) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Graph Operations
  // ═══════════════════════════════════════════════════════════════════════════

  /// Add a relationship edge in the graph.
  void add_relationship(const std::string &from_guid,
                        const std::string &to_guid,
                        const std::string &relationship_type = "default");

  /// Get the neighbours of a GUID
  std::vector<std::string> get_neighbours(const std::string &obj_guid);

  // ═══════════════════════════════════════════════════════════════════════════
  // SpatialBVH Collision Detection
  // ═══════════════════════════════════════════════════════════════════════════

  /// Bounding box of an object in WORLD placement, inflated by tolerance.
  static OBB compute_bounding_box(const Geometry& geometry, const Xform& xform);

  /// Get all collision pairs using SpatialBVH and add them as graph edges.
  std::vector<std::pair<std::string, std::string>> get_collisions();

  // ═══════════════════════════════════════════════════════════════════════════
  // Ray Intersection
  // ═══════════════════════════════════════════════════════════════════════════

  /// Result of a ray intersection with geometry.
  struct RayHit {
    std::string guid;      ///< GUID of hit object
    Point hit_point;       ///< Intersection point
    double distance;        ///< Distance from ray origin
  };

  /// Cast a ray through the scene and find the closest intersecting geometry.
  std::vector<RayHit> ray_cast(const Point& origin, const Vector& direction, double tolerance = 1e-3);

  // ═══════════════════════════════════════════════════════════════════════════
  // Transformed Geometry
  // ═══════════════════════════════════════════════════════════════════════════

  /// All geometry with its hierarchical placement BAKED into the coordinates.
  Objects get_geometry() const;

  // ═══════════════════════════════════════════════════════════════════════════
  // JSON Serialization
  // ═══════════════════════════════════════════════════════════════════════════

  /// Serializes the Session instance to JSON.
  nlohmann::ordered_json jsondump() const;

  /// Creates a Session instance from JSON data.
  static Session jsonload(const nlohmann::json &data);
  std::string file_json_dumps() const;
  static Session file_json_loads(const std::string& json_string);
  void file_json_dump(const std::string& filename) const;
  static Session file_json_load(const std::string& filename);
  std::string pb_dumps() const;
  static Session pb_loads(const std::string& data);
  void pb_dump(const std::string& filename) const;
  static Session pb_load(const std::string& filename);

private:
  friend class History;
  mutable std::string _guid;

  /// Store an object in its typed list, lookup, graph and tree, recording an AddOp when a
  /// transaction is open.
  std::shared_ptr<TreeNode> _add_object(const std::string &collection, const Item &obj, const std::string &type_prefix, std::shared_ptr<TreeNode> parent);

  /// Which Objects list holds a guid, and where; ("", -1) when none does.
  std::pair<std::string, int> _locate(const std::string &guid) const;

  /// Take an object out of every live table, unrecorded, returning its tombstone.
  std::optional<RemoveOp> _detach(const std::string &guid);

  /// Put an object back from its tombstone, unrecorded: typed list at its old index, lookup,
  /// xform, tree node under the same parent at the same index with its subtree, graph node and
  /// every incident edge whose other end is still present.
  void _attach(const Tombstone &op);

  /// Store obj under guid in its typed list and lookup, unrecorded.
  void _swap(const std::string &guid, const Item &obj);

  /// Point `lookup` and `component_lookup` at the objects this session currently holds. Used
  /// after anything that replaces the collections wholesale: a load, or a copy.
  void _index_objects();

  /// Set or drop (nullopt) the local transform under guid, unrecorded.
  void _place(const std::string &guid, const std::optional<Xform> &xform);

  /// The xforms in canonical order() sequence, identity entries omitted - the exact sequence
  /// jsondump and pb_dumps write, so both formats share one order.
  std::vector<std::pair<std::string, Xform>> xforms_ordered() const;

  /// Test ray intersection with a specific geometry object.
  std::optional<Point> ray_intersect_geometry(const Line& ray, const Geometry& geometry, double tolerance, const Xform& placement);
  
  /// Rebuild the cached SpatialBVH for ray casting.
  void rebuild_ray_bvh_cache();
  
  /// Invalidate the SpatialBVH cache (call when geometry is added/removed).
  void invalidate_bvh_cache() { bvh_cache_dirty = true; }
  
  /// Cache geometry bounding box incrementally
  void cache_geometry_aabb(const std::string& obj_guid, const Geometry& geometry);
};
/// To use this operator, you can do:
std::ostream &operator<<(std::ostream &os, const Session &session);
} // namespace session_cpp
