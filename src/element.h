#pragma once
#include "guid.h"
#include "json.h"
#include "mesh.h"
#include "brep.h"
#include "obb.h"
#include "xform.h"
#include "line.h"
#include "plane.h"
#include "point.h"
#include "polyline.h"
#include "vector.h"
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace session_proto {
class Element;
class ElementFeature;
}

namespace session_cpp {

/// One serializable modification of a host element - a cut, a drill, a joint pocket - that the kernel draws but never applies.
struct ElementFeature {
private:
    mutable std::string _guid; // Lazily minted guid.

public:
    std::string name; // Feature name.
    std::string feature_type; // The package's vocabulary: "cut", "drill", "joint".
    int face_index = -1; // Face of the host this applies to; -1 = whole element.
    std::vector<Polyline> outlines; // Closed outlines that bound the feature.
    bool visible = true; // Whether a viewer draws it.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty feature.
    ElementFeature() = default;

    /// Construct from type, host face, outlines and name.
    ElementFeature(std::string feature_type, int face_index, std::vector<Polyline> outlines, std::string name = "")
        : name(std::move(name)), feature_type(std::move(feature_type)), face_index(face_index),
          outlines(std::move(outlines)) {}

    /// Copy with a new guid and the same data.
    ElementFeature(const ElementFeature& other)
        : name(other.name), feature_type(other.feature_type), face_index(other.face_index), outlines(other.outlines),
          visible(other.visible) {}

    /// Copy-assign with a new guid and the same data.
    ElementFeature& operator=(const ElementFeature& other);

    /// Move while preserving the guid.
    ElementFeature(ElementFeature&& other) noexcept = default;

    /// Move-assign while preserving the guid.
    ElementFeature& operator=(ElementFeature&& other) noexcept = default;

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    bool has_guid() const { return !_guid.empty(); }

    /// Return the guid, creating it on first access.
    const std::string& guid() const {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return the mutable guid, creating it on first access.
    std::string& guid() {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Clear the guid so a fresh one mints lazily on the next read.
    void refresh_guid() { _guid.clear(); }

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Compare name, type, face, outlines and visibility; guid ignored.
    bool operator==(const ElementFeature& other) const;

    /// Compare name, type, face, outlines and visibility; guid ignored.
    bool operator!=(const ElementFeature& other) const { return !(*this == other); }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static ElementFeature jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static ElementFeature file_json_loads(const std::string& json_string);

    /// Write to a JSON file.
    void file_json_dump(const std::string& filename) const;

    /// Read from a JSON file.
    static ElementFeature file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    session_proto::ElementFeature to_proto() const;

    /// Construct from the protobuf message.
    static ElementFeature from_proto(const session_proto::ElementFeature& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static ElementFeature pb_loads(const std::string& data);

    /// Write to a protobuf file.
    void pb_dump(const std::string& filename) const;

    /// Read from a protobuf file.
    static ElementFeature pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return a string representation of the feature.
    std::string str() const;

    /// Return a string representation of the feature for debugging.
    std::string repr() const;

    /// Write the feature string to a stream.
    friend std::ostream& operator<<(std::ostream& os, const ElementFeature& f);
};

/// Named geometry carrier with lazily cached boxes, features and a polymorphic type registry.
class Element {
private:
    mutable std::string _guid; // Lazily minted guid.

protected:
    std::optional<Mesh> _geometry_mesh; // Mesh stored for the session and serialization.
    std::optional<BRep> _geometry_brep; // BRep stored for the session and serialization.
    mutable std::optional<Mesh> _model_mesh_cache; // Model mesh with in-memory operations applied.
    mutable std::optional<BRep> _model_brep_cache; // Model BRep, cached independently of the mesh.
    mutable bool _geometry_synced = false; // Whether the slot holds what compute_geometry_mesh() would write.
    mutable bool _computing_geometry = false; // Guards ensure_geometry() against re-entry from compute_geometry_mesh().
    bool _is_dirty = true; // Whether the caches must be recomputed.
    std::optional<OBB> _aabb; // Cached axis-aligned box.
    std::optional<OBB> _obb; // Cached oriented box.
    std::optional<Mesh> _collision_mesh; // Cached collision mesh.
    std::optional<Point> _point; // Cached centroid.
    std::optional<std::vector<Polyline>> _polylines; // Cached face outlines.
    std::optional<std::vector<Plane>> _planes; // Cached face planes.
    std::optional<std::vector<Vector>> _edge_vectors; // Cached edge directions.
    std::optional<Line> _axis; // Cached main axis.
    std::vector<std::function<Mesh(Mesh)>> _geometry_ops; // In-memory mesh operations, never written.
    std::vector<ElementFeature> _features; // Serialized modifications.
    std::vector<Vector> _insertion_vectors; // One insertion direction per jointed face.
    std::optional<Vector> _dimensions; // Authored nominal extents.
    std::string _element_type; // Derived type name this element was loaded with.
    std::string _element_data; // Opaque derived-type state.

public:
    std::string name; // Element name.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty element with a name.
    Element(const std::string& name = "my_element");

    /// Construct from a mesh with a name.
    Element(const Mesh& geometry, const std::string& name = "my_element");

    /// Construct from a BRep with a name.
    Element(const BRep& geometry, const std::string& name = "my_element");

    /// Copy with a new guid and the same data.
    Element(const Element& other);

    /// Copy-assign with a new guid and the same data.
    Element& operator=(const Element& other);

    /// Move while preserving the guid.
    Element(Element&& other) noexcept = default;

    /// Move-assign while preserving the guid.
    Element& operator=(Element&& other) noexcept = default;

    /// Destroy the element.
    virtual ~Element() = default;

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    bool has_guid() const { return !_guid.empty(); }

    /// Return the guid, creating it on first access.
    const std::string& guid() const {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return the mutable guid, creating it on first access.
    std::string& guid() {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Clear the guid so a fresh one mints lazily on the next read.
    void refresh_guid() { _guid.clear(); }

    /// Return the element's mesh before modifications; empty when no mesh exists. Domain types override this with their lazy parametric mesh.
    virtual const Mesh& element_geometry_mesh() const;

    /// Return the element's BRep before modifications; empty when no BRep exists. Domain types override this with their lazy parametric BRep.
    virtual const BRep& element_geometry_brep() const;

    /// Return the model mesh with in-memory operations applied, cached until invalidation. Domain types override this to apply their joints or cuts.
    virtual const Mesh& model_geometry_mesh() const;

    /// Return the model BRep, cached independently until invalidation. Domain types override this to apply their joints or cuts.
    virtual const BRep& model_geometry_brep() const;

    /// Return the local mesh, computing it on demand; empty when this element has no mesh.
    const Mesh& geometry_mesh() const;

    /// Return the local BRep, computing it on demand; empty when this element has no BRep.
    const BRep& geometry_brep() const;

    /// Write the element's mesh, features and dimensions into the session slot, reusing a current mesh.
    void compute_geometry_mesh();

    /// Write the element's BRep, features and dimensions into the session slot, reusing a current BRep.
    void compute_geometry_brep();

    /// Return whether the slot already holds what compute_geometry_mesh() would write.
    bool geometry_synced() const { return _geometry_synced; }

    /// Mark the slot stale, so the next read computes it again; a domain type overrides this to drop its own caches too.
    virtual void invalidate_geometry() {

        _geometry_synced = false;
        _model_mesh_cache.reset();
        _model_brep_cache.reset();
    }

    /// Return whether the element carries a mesh or a BRep.
    bool has_geometry() const;

    /// Return "Mesh", "BRep" or "None".
    std::string geometry_type_name() const;

    /// Return the mesh with in-memory operations and placement applied, empty when no mesh exists.
    Mesh session_geometry_mesh(const Xform& xform) const;

    /// Return the BRep with placement applied, empty when no BRep exists.
    BRep session_geometry_brep(const Xform& xform) const;

    /// Return the cached axis-aligned box, computing it when dirty.
    OBB aabb();

    /// Return the cached oriented box, computing it when dirty.
    OBB obb();

    /// Return the cached collision mesh, computing it when dirty.
    Mesh collision_mesh();

    /// Return the cached centroid, computing it when dirty.
    Point point();

    /// Return the cached face outlines, computing them when dirty.
    std::vector<Polyline> polylines();

    /// Return the cached face planes, computing them when dirty.
    std::vector<Plane> planes();

    /// Return the cached edge directions, computing them when dirty.
    std::vector<Vector> edge_vectors();

    /// Return the cached main axis, computing it when dirty.
    std::optional<Line> axis();

    /// Return whether the caches must be recomputed.
    bool is_dirty() const { return _is_dirty; }

    /// Return the cached axis-aligned box without computing it.
    const std::optional<OBB>& cached_aabb() const { return _aabb; }

    /// Return the cached oriented box without computing it.
    const std::optional<OBB>& cached_obb() const { return _obb; }

    /// Return the cached collision mesh without computing it.
    const std::optional<Mesh>& cached_collision_mesh() const { return _collision_mesh; }

    /// Return the cached centroid without computing it.
    const std::optional<Point>& cached_point() const { return _point; }

    /// Return the number of in-memory geometry operations.
    size_t geometry_ops_count() const { return _geometry_ops.size(); }

    /// Return the number of features.
    size_t features_count() const { return _features.size(); }

    /// Return the modifications carried by this element and written with it; add_geometry_op is the in-memory counterpart that is not.
    const std::vector<ElementFeature>& features() const {

        ensure_geometry();

        return _features;
    }

    /// Return the directions the element is inserted along when the assembly is put together, one per jointed face.
    const std::vector<Vector>& insertion_vectors() const { return _insertion_vectors; }

    /// Return the nominal extents in the element's own frame (plate: x/y outline, z thickness), authored intent rather than the measured obb; nullopt = never authored.
    const std::optional<Vector>& dimensions() const {

        ensure_geometry();

        return _dimensions;
    }

    /// Return the derived type name this element was loaded with, written to element_type; a plain Element authored in memory returns "".
    virtual std::string element_type_name() const { return _element_type; }

    /// Return the derived type's own state, opaque to the kernel and carried through untouched.
    virtual std::string element_data_dumps() const { return _element_data; }

    // ═══════════════════════════════════════════════════════════════════════════
    // Mutators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Append an in-memory mesh operation and invalidate the caches.
    void add_geometry_op(std::function<Mesh(Mesh)> f);

    /// Replace the features.
    void set_features(std::vector<ElementFeature> features) { _features = std::move(features); }

    /// Append a feature.
    void add_feature(ElementFeature feature) { _features.push_back(std::move(feature)); }

    /// Replace the insertion vectors.
    void set_insertion_vectors(std::vector<Vector> v) { _insertion_vectors = std::move(v); }

    /// Set the nominal extents.
    void set_dimensions(const Vector& d) { _dimensions = d; }

    /// Bake a placement into the geometry, the feature outlines and the insertion vectors, then drop the caches; a domain type overrides it to move its own members too.
    virtual void place(const Xform& xform);

    /// Replace the geometry with a mesh and invalidate the caches.
    void set_geometry(const Mesh& geo);

    /// Replace the geometry with a BRep and invalidate the caches.
    void set_geometry(const BRep& geo);

    /// Override the cached face outlines, kept until the next reset.
    void set_polylines(std::vector<Polyline> polys);

    /// Override the cached face planes, kept until the next reset.
    void set_planes(std::vector<Plane> plns);

    /// Drop every cache and mark the element dirty.
    void reset();

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Compare every field that survives a round trip; guid ignored.
    virtual bool operator==(const Element& other) const;

    /// Compare every field that survives a round trip; guid ignored.
    bool operator!=(const Element& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Utilities
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return a copy with a new guid.
    Element duplicate() const;

    /// Return a polymorphic copy of the same derived type; a derived class overrides it with one line.
    virtual std::shared_ptr<Element> clone() const { return std::make_shared<Element>(*this); }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    virtual nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Element jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static Element file_json_loads(const std::string& s);

    /// Write to a JSON file.
    void file_json_dump(const std::string& path) const;

    /// Read from a JSON file.
    static Element file_json_load(const std::string& path);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    session_proto::Element to_proto() const;

    /// Construct from the protobuf message.
    static Element from_proto(const session_proto::Element& proto);

    /// Serialize to protobuf bytes.
    virtual std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static Element pb_loads(const std::string& data);

    /// Write to a protobuf file.
    void pb_dump(const std::string& path) const;

    /// Read from a protobuf file.
    static Element pb_load(const std::string& path);

    // ═══════════════════════════════════════════════════════════════════════════
    // Polymorphic registry
    // ═══════════════════════════════════════════════════════════════════════════
    /// Build one element from full serialized session_proto.Element bytes, so a factory reads the base fields as well as element_data.
    using Factory = std::function<std::shared_ptr<Element>(const std::string& data)>;

    /// Register factory for type_name; re-registering the same name replaces it.
    static void register_type(const std::string& type_name, Factory factory);

    /// Return whether a factory is registered for type_name.
    static bool is_registered(const std::string& type_name);

    /// Return the registered type names.
    static std::vector<std::string> registered_types();

    /// Load through the registered factory, degrading to a base Element that still carries element_type and element_data when the type is unknown or the factory fails.
    static std::shared_ptr<Element> pb_loads_polymorphic(const std::string& data);

    /// Load from JSON through the registered factory, re-encoded to proto bytes so one registration serves both formats.
    static std::shared_ptr<Element> file_json_loads_polymorphic(const std::string& s);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return a string representation of the element.
    virtual std::string str() const;

    /// Return a string representation of the element for debugging.
    virtual std::string repr() const;

    /// Write the element string to a stream.
    friend std::ostream& operator<<(std::ostream& os, const Element& e);

protected:
    // ═══════════════════════════════════════════════════════════════════════════
    // Computation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Run compute_geometry_mesh() once while the slot is stale, so every reader and the file see the current solid, features and dimensions.
    void ensure_geometry() const {

        if (_geometry_synced || _computing_geometry)
            return;

        const_cast<Element*>(this)->compute_geometry_mesh();
    }

    /// Compute the mesh, features and dimensions; a domain type overrides this.
    virtual void compute_geometry_mesh_impl() {}

    /// Compute the BRep, features and dimensions; a domain type overrides this.
    virtual void compute_geometry_brep_impl() {}

    /// Compute the axis-aligned box of the placed geometry.
    OBB compute_aabb();

    /// Compute the oriented box of the placed geometry.
    OBB compute_obb();

    /// Compute the collision mesh; a BRep yields an empty mesh.
    Mesh compute_collision_mesh();

    /// Compute the centroid of the placed geometry.
    Point compute_point();

    /// Compute the face outlines of a mesh solid; a domain type with its own face order overrides this.
    virtual std::vector<Polyline> compute_polylines() const;

    /// Compute one plane per face outline: centroid origin, Newell normal, closing point dropped first.
    virtual std::vector<Plane> compute_planes() const;

    /// Compute the edge directions; the base element has none.
    virtual std::vector<Vector> compute_edge_vectors() const;

    /// Compute the main axis; the base element has none.
    virtual std::optional<Line> compute_axis() const;

    /// Run the in-memory operations over a mesh.
    Mesh apply_geometry_ops(Mesh geo) const;

    /// Return the stored geometry vertices with in-memory mesh operations applied.
    std::vector<Point> geometry_points() const;
};

/// Write the element string to a stream.
std::ostream& operator<<(std::ostream& os, const Element& e);

/// Write the feature string to a stream.
std::ostream& operator<<(std::ostream& os, const ElementFeature& f);

} // namespace session_cpp
