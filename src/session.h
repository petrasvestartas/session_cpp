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
    std::string name = "my_session";                             // The name of the session.
    Objects objects;                                             // Collection of geometry objects.
    std::unordered_map<std::string, Geometry> lookup;            // Fast lookup table for geometry by GUID.
    Tree tree;                                                   // Tree structure for hierarchy.
    Graph graph;                                                 // Graph structure for relationships.
    std::unordered_map<std::string, Component> component_lookup; // Fast lookup table for components by GUID.
    std::unordered_map<std::string, Xform> xforms;               // LOCAL transform per guid, relative to the parent.
    mutable History history;                                     // Undo/redo buffer, in memory only; every save purges it.
    SpatialBVH bvh;                                              // Bounding volume hierarchy for collision detection.
    SpatialBVH cached_ray_bvh;                                   // Cached SpatialBVH for ray casting.
    std::vector<std::string> cached_guids;                       // GUID per leaf of cached_ray_bvh.
    std::vector<OBB> cached_boxes;                               // Box per leaf of cached_ray_bvh.
    bool bvh_cache_dirty = true;                                 // Flag to rebuild cached_ray_bvh.

    /// Constructs an empty session whose tree root carries the session name.
    Session(std::string name = "my_session");

    /// Copy every table and object, guids included; caches are rebuilt on demand and history starts empty.
    Session(const Session& other);

    /// Copy-assign every table and object, guids included.
    Session& operator=(const Session& other);

    /// Move the tables as they are.
    Session(Session&&) noexcept = default;

    /// Move-assign the tables as they are.
    Session& operator=(Session&&) noexcept = default;

    /// Return whether the lazy guid has been created.
    bool has_guid() const { return !_guid.empty(); }

    /// Return the guid, creating it on first access.
    const std::string& guid() const {
        if (_guid.empty())

            /// Return the mutable guid, creating it on first access.
            _guid = ::guid();

        return _guid;
    }

    /// Return the mutable guid, creating it on first access.
    std::string& guid() {
        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════

    /// Get a geometry object by GUID with type safety.
    template <typename T> std::shared_ptr<T> get_object(const std::string& guid) {

        auto it = lookup.find(guid);

        if (it == lookup.end())
            return nullptr;

        const std::shared_ptr<T>* ptr = std::get_if<std::shared_ptr<T>>(&it->second);

        return ptr ? *ptr : nullptr;
    }

    /// Get a geometry object by GUID (const version).
    template <typename T> std::shared_ptr<const T> get_object(const std::string& guid) const {

        auto it = lookup.find(guid);

        if (it == lookup.end())
            return nullptr;

        const std::shared_ptr<T>* ptr = std::get_if<std::shared_ptr<T>>(&it->second);

        return ptr ? *ptr : nullptr;
    }

    /// Select objects of one type, grouped by the top-level nodes of the tree.
    template <typename T> std::vector<std::vector<T>> select_by_type() const {

        std::vector<std::vector<T>> groups;
        std::shared_ptr<TreeNode> root = tree.root();

        if (!root)
            return groups;

        for (TreeNode* group : root->children()) {
            std::vector<T> items;

            for (TreeNode* node : group->descendants())
                if (std::shared_ptr<const T> object = get_object<T>(node->name))
                    items.push_back(*object);

            if (!items.empty())
                groups.push_back(std::move(items));
        }

        return groups;
    }

    /// Find an existing group by name; throws std::runtime_error when there is none.
    std::shared_ptr<TreeNode> find_group(const std::string& group_name) const;

    /// Canonical object order: the objects lists walked in one fixed type sequence.
    std::vector<std::string> order() const;

    /// The LOCAL transform of an object, identity when none was set.
    Xform xform(const std::string& guid) const;

    /// The CUMULATIVE placement of an object: every ancestor's transform multiplied down the tree onto its own.
    Xform world_xform(const std::string& guid) const;

    /// Every object's cumulative placement, computed in one downward pass.
    std::unordered_map<std::string, Xform> world_xforms() const;

    /// Get the children of a parent GUID.
    std::vector<std::string> get_children(const std::string& obj_guid) const;

    /// Get the neighbours of a GUID.
    std::vector<std::string> get_neighbours(const std::string& obj_guid);

    /// All geometry with its hierarchical placement BAKED into the coordinates.
    Objects get_geometry() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry management
    // ═══════════════════════════════════════════════════════════════════════════

    /// Add a point; null adds nothing and returns nullptr.
    std::shared_ptr<TreeNode> add_point(std::shared_ptr<Point> point, std::shared_ptr<TreeNode> parent = nullptr);

    /// Add a line; null adds nothing and returns nullptr.
    std::shared_ptr<TreeNode> add_line(std::shared_ptr<Line> line, std::shared_ptr<TreeNode> parent = nullptr);

    /// Add a plane; null adds nothing and returns nullptr.
    std::shared_ptr<TreeNode> add_plane(std::shared_ptr<Plane> plane, std::shared_ptr<TreeNode> parent = nullptr);

    /// Add a bounding box; null adds nothing and returns nullptr.
    std::shared_ptr<TreeNode> add_obb(std::shared_ptr<OBB> bbox);

    /// Add a polyline; null, or fewer than two points, adds nothing and returns nullptr.
    std::shared_ptr<TreeNode> add_polyline(
        std::shared_ptr<Polyline> polyline,
        std::shared_ptr<TreeNode> parent = nullptr
    );

    /// Add a point cloud; null, or no points, adds nothing and returns nullptr.
    std::shared_ptr<TreeNode> add_pointcloud(
        std::shared_ptr<PointCloud> pointcloud,
        std::shared_ptr<TreeNode> parent = nullptr
    );

    /// Add a mesh; null, or no faces, adds nothing and returns nullptr.
    std::shared_ptr<TreeNode> add_mesh(std::shared_ptr<Mesh> mesh, std::shared_ptr<TreeNode> parent = nullptr);

    /// Add a curve; null, or fewer than two control vertices, adds nothing and returns nullptr.
    std::shared_ptr<TreeNode> add_nurbscurve(
        std::shared_ptr<NurbsCurve> nurbscurve,
        std::shared_ptr<TreeNode> parent = nullptr
    );

    /// Add a surface; null, or no control vertices, adds nothing and returns nullptr.
    std::shared_ptr<TreeNode> add_nurbssurface(
        std::shared_ptr<NurbsSurface> nurbssurface,
        std::shared_ptr<TreeNode> parent = nullptr
    );

    /// Add a brep; null, or no faces and no vertices, adds nothing and returns nullptr.
    std::shared_ptr<TreeNode> add_brep(std::shared_ptr<BRep> brep, std::shared_ptr<TreeNode> parent = nullptr);

    /// Add an element; only null adds nothing, an Element is a data record kept even without geometry.
    std::shared_ptr<TreeNode> add_element(std::shared_ptr<Element> element, std::shared_ptr<TreeNode> parent = nullptr);

    /// Add a custom component (any object with type_name/guid/name/extra).
    std::shared_ptr<TreeNode> add_component(Component component, std::shared_ptr<TreeNode> parent = nullptr);

    /// Add a TreeNode to the tree hierarchy, under the root when no parent is given; null is ignored.
    void add(std::shared_ptr<TreeNode> node, std::shared_ptr<TreeNode> parent = nullptr);

    /// Create a named group (TreeNode) and add it to the root of the tree.
    std::shared_ptr<TreeNode> add_group(const std::string& group_name);

    /// Add an edge between two geometry objects in the graph.
    void add_edge(const std::string& guid1, const std::string& guid2, const std::string& attribute = "");

    /// Add a parent-child relationship in the tree.
    bool add_hierarchy(const std::string& parent_guid, const std::string& child_guid);

    /// Add a relationship edge in the graph.
    void add_relationship(
        const std::string& from_guid,
        const std::string& to_guid,
        const std::string& relationship_type = "default"
    );

    /// Remove an object by its GUID from every live table at once; the removal record is the tombstone undo restores from.
    bool remove_object(const std::string& obj_guid);

    /// Swap the object stored under guid for obj, which takes over that guid; the recorded edit undo and redo restore as absolute snapshots.
    bool replace(const std::string& guid, const Geometry& obj);

    /// Sets the LOCAL transform of an object, relative to its tree parent.
    void set_xform(const std::string& guid, const Xform& xform);

    /// Removes an object's local transform, returning whether one was present.
    bool remove_xform(const std::string& guid);

    // ═══════════════════════════════════════════════════════════════════════════
    // History
    // ═══════════════════════════════════════════════════════════════════════════

    /// Open a history transaction: every add, remove, replace and xform change until commit becomes one undo step.
    void begin(const std::string& label);

    /// Close the open transaction as one undo step.
    void commit();

    /// Revert the latest committed transaction, returning whether there was one.
    bool undo();

    /// Reapply the latest undone transaction, returning whether there was one.
    bool redo();

    // ═══════════════════════════════════════════════════════════════════════════
    // Collision detection and ray casting
    // ═══════════════════════════════════════════════════════════════════════════

    /// Bounding box of an object in WORLD placement, inflated by tolerance.
    static OBB compute_bounding_box(const Geometry& geometry, const Xform& xform);

    /// Get all collision pairs using SpatialBVH and add them as graph edges.
    std::vector<std::pair<std::string, std::string>> get_collisions();

    /// One object a ray touched: which one, where, and how far from the ray origin.
    struct RayHit {
        std::string guid; // GUID of the hit object.
        Point hit_point;  // Intersection point in world coordinates.
        double distance;  // Distance from the ray origin.
    };

    /// Cast a ray through the scene, returning the hits within tolerance of the closest one.
    std::vector<RayHit> ray_cast(const Point& origin, const Vector& direction, double tolerance = 1e-3);

    // ═══════════════════════════════════════════════════════════════════════════
    // Serialization
    // ═══════════════════════════════════════════════════════════════════════════

    /// Serialize to a JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Session jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static Session file_json_loads(const std::string& json_string);

    /// Write to a JSON file.
    void file_json_dump(const std::string& filename) const;

    /// Read from a JSON file.
    static Session file_json_load(const std::string& filename);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static Session pb_loads(const std::string& data);

    /// Write to a protobuf file.
    void pb_dump(const std::string& filename) const;

    /// Read from a protobuf file.
    static Session pb_load(const std::string& filename);

    /// Return a string representation of the session.
    std::string str() const;

private:
    friend class History;
    mutable std::string _guid; // Lazily minted guid.

    /// Store an object in its typed list, lookup, graph and tree, recording an AddOp when a transaction is open.
    std::shared_ptr<TreeNode> _add_object(
        const std::string& collection,
        const Item& obj,
        const std::string& type_prefix,
        std::shared_ptr<TreeNode> parent
    );

    /// Which Objects list holds a guid, and where; ("", -1) when none does.
    std::pair<std::string, int> _locate(const std::string& guid) const;

    /// Take an object out of every live table, unrecorded, returning its tombstone.
    std::optional<RemoveOp> _detach(const std::string& guid);

    /// Put an object back from its tombstone, unrecorded: typed list, lookup, xform, tree node with its subtree, graph node and edges.
    void _attach(const Tombstone& op);

    /// Store obj under guid in its typed list and lookup, unrecorded.
    void _swap(const std::string& guid, const Item& obj);

    /// Point lookup and component_lookup at the objects this session currently holds.
    void _index_objects();

    /// Set or drop (nullopt) the local transform under guid, unrecorded.
    void _place(const std::string& guid, const std::optional<Xform>& xform);

    /// The xforms in canonical order() sequence, identity entries omitted, the exact sequence jsondump and pb_dumps write.
    std::vector<std::pair<std::string, Xform>> _xforms_ordered() const;

    /// World bounding box of every object in order() sequence, with the guid of each.
    std::vector<OBB> _compute_boxes(std::vector<std::string>& guids) const;

    /// Rebuild the cached SpatialBVH for ray casting.
    void _rebuild_ray_bvh_cache();

    /// Test ray intersection with a specific geometry object, returning the world hit.
    std::optional<Point> _ray_intersect_geometry(
        const Line& ray,
        const Geometry& geometry,
        double tolerance,
        const Xform& placement
    ) const;
};

/// Write the session string to a stream.
std::ostream& operator<<(std::ostream& os, const Session& session);
} // namespace session_cpp
