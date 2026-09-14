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
#include <variant>
#include <vector>

namespace session_cpp {

using ElementGeometry = std::variant<std::monostate, Mesh, BRep>;

/// One serializable modification of a host element - a cut, a drill, a joint pocket - that the kernel draws but never applies.
struct ElementFeature {
    std::string name;
    std::string feature_type;       ///< The package's vocabulary: "cut", "drill", "joint"
    int face_index = -1;            ///< Face of the host this applies to; -1 = whole element
    std::vector<Polyline> outlines;

    ElementFeature() = default;
    ElementFeature(std::string feature_type, int face_index, std::vector<Polyline> outlines, std::string name = "")
        : name(std::move(name)), feature_type(std::move(feature_type)), face_index(face_index), outlines(std::move(outlines)) {}
    /// A copy is a new feature and mints its own guid; a move keeps it.
    ElementFeature(const ElementFeature& other)
        : name(other.name), feature_type(other.feature_type), face_index(other.face_index), outlines(other.outlines) {}
    ElementFeature& operator=(const ElementFeature& other);
    ElementFeature(ElementFeature&& other) noexcept = default;
    ElementFeature& operator=(ElementFeature&& other) noexcept = default;

    bool has_guid() const { return !_guid.empty(); }
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
    /// Clear the guid so a fresh one mints lazily on next read.
    void refresh_guid() { _guid.clear(); }

    /// Data equality, not identity: the guid is ignored.
    bool operator==(const ElementFeature& other) const;
    bool operator!=(const ElementFeature& other) const { return !(*this == other); }

    nlohmann::ordered_json jsondump() const;
    static ElementFeature jsonload(const nlohmann::json& data);
    std::string file_json_dumps() const;
    static ElementFeature file_json_loads(const std::string& json_string);
    void file_json_dump(const std::string& filename) const;
    static ElementFeature file_json_load(const std::string& filename);

    std::string pb_dumps() const;
    static ElementFeature pb_loads(const std::string& data);
    void pb_dump(const std::string& filename) const;
    static ElementFeature pb_load(const std::string& filename);

    std::string str() const;
    std::string repr() const;
    friend std::ostream& operator<<(std::ostream& os, const ElementFeature& f);

private:
    mutable std::string _guid;
};

class Element {
public:
    std::string name;

    Element(const std::string& name = "my_element");
    Element(const Mesh& geometry, const std::string& name = "my_element");
    Element(const BRep& geometry, const std::string& name = "my_element");
    /// A copy is a new element and mints its own guid; a move keeps it.
    Element(const Element& other);
    Element& operator=(const Element& other);
    Element(Element&& other) noexcept = default;
    Element& operator=(Element&& other) noexcept = default;
    virtual ~Element() = default;

    bool has_guid() const { return !_guid.empty(); }
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
    const ElementGeometry& geometry() const { return _geometry; }
    bool has_geometry() const;
    std::string geometry_type_name() const;
    /// The geometry placed by `xform`; the Session owns the placement, so pass identity for local geometry.
    ElementGeometry session_geometry(const Xform& xform) const;
    OBB aabb();
    OBB obb();
    Mesh collision_mesh();
    Point point();
    std::vector<Polyline> polylines();
    std::vector<Plane> planes();
    std::vector<Vector> edge_vectors();
    std::optional<Line> axis();
    bool is_dirty() const { return _is_dirty; }
    const std::optional<OBB>& cached_aabb() const { return _aabb; }
    const std::optional<OBB>& cached_obb() const { return _obb; }
    const std::optional<Mesh>& cached_collision_mesh() const { return _collision_mesh; }
    const std::optional<Point>& cached_point() const { return _point; }
    size_t geometry_ops_count() const { return _geometry_ops.size(); }
    size_t features_count() const { return _features.size(); }
    /// Modifications carried by this element and written with it; `add_geometry_op` is the in-memory counterpart that is not.
    const std::vector<ElementFeature>& features() const { return _features; }
    /// Direction(s) the element is inserted along when the assembly is put together, one per jointed face.
    const std::vector<Vector>& insertion_vectors() const { return _insertion_vectors; }
    /// Nominal extents in the element's own frame (plate: x/y outline, z thickness) - authored intent, not the measured `obb()`; nullopt = never authored.
    const std::optional<Vector>& dimensions() const { return _dimensions; }
    /// The derived type name this element was loaded with, written to `element_type`; a plain Element authored in memory returns "".
    virtual std::string element_type_name() const { return _element_type; }
    /// The derived type's own state, opaque to the kernel and carried through untouched.
    virtual std::string element_data_dumps() const { return _element_data; }

    void add_geometry_op(std::function<Mesh(Mesh)> f);
    void set_features(std::vector<ElementFeature> features) { _features = std::move(features); }
    void add_feature(ElementFeature feature) { _features.push_back(std::move(feature)); }
    void set_insertion_vectors(std::vector<Vector> v) { _insertion_vectors = std::move(v); }
    void set_dimensions(const Vector& d) { _dimensions = d; }
    /// Bake a placement into the element's own geometry, invalidating the cached boxes.
    void place(const Xform& xform);
    void set_geometry(const Mesh& geo);
    void set_geometry(const BRep& geo);
    void set_polylines(std::vector<Polyline> polys);
    void set_planes(std::vector<Plane> plns);
    void reset();

    /// Data equality, not identity: every field that survives a round trip, guid excluded.
    virtual bool operator==(const Element& other) const;
    bool operator!=(const Element& other) const;

    Element duplicate() const;
    /// A polymorphic copy of the same derived type; a derived class overrides it with one line.
    virtual std::shared_ptr<Element> clone() const { return std::make_shared<Element>(*this); }

    virtual nlohmann::ordered_json jsondump() const;
    static Element jsonload(const nlohmann::json& data);
    std::string file_json_dumps() const;
    static Element file_json_loads(const std::string& s);
    void file_json_dump(const std::string& path) const;
    static Element file_json_load(const std::string& path);

    virtual std::string pb_dumps() const;
    static Element pb_loads(const std::string& data);
    void pb_dump(const std::string& path) const;
    static Element pb_load(const std::string& path);

    /// Builds one element from full serialized `session_proto.Element` bytes, so a factory reads the base fields as well as `element_data`.
    using Factory = std::function<std::shared_ptr<Element>(const std::string& data)>;
    /// Register `factory` for `type_name`; re-registering the same name replaces it.
    static void register_type(const std::string& type_name, Factory factory);
    static bool is_registered(const std::string& type_name);
    static std::vector<std::string> registered_types();
    /// Load through the registered factory, degrading to a base Element that still carries `element_type`/`element_data` when the type is unknown or the factory fails.
    static std::shared_ptr<Element> pb_loads_polymorphic(const std::string& data);
    /// The same from JSON, re-encoded to proto bytes so one registration serves both formats.
    static std::shared_ptr<Element> file_json_loads_polymorphic(const std::string& s);

    virtual std::string str() const;
    virtual std::string repr() const;
    friend std::ostream& operator<<(std::ostream& os, const Element& e);

private:
    mutable std::string _guid;

protected:
    ElementGeometry _geometry;
    bool _is_dirty = true;
    std::optional<OBB> _aabb;
    std::optional<OBB> _obb;
    std::optional<Mesh> _collision_mesh;
    std::optional<Point> _point;
    std::optional<std::vector<Polyline>> _polylines;
    std::optional<std::vector<Plane>> _planes;
    std::optional<std::vector<Vector>> _edge_vectors;
    std::optional<Line> _axis;
    std::vector<std::function<Mesh(Mesh)>> _geometry_ops;
    std::vector<ElementFeature> _features;
    std::vector<Vector> _insertion_vectors;
    std::optional<Vector> _dimensions;
    std::string _element_type;
    std::string _element_data;

    OBB compute_aabb();
    OBB compute_obb();
    Mesh compute_collision_mesh();
    Point compute_point();
    virtual std::vector<Polyline> compute_polylines() const;
    virtual std::vector<Plane> compute_planes() const;
    virtual std::vector<Vector> compute_edge_vectors() const;
    virtual std::optional<Line> compute_axis() const;
    Mesh apply_geometry_ops(Mesh geo) const;
    static std::vector<Point> points_from_geometry(const ElementGeometry& geo);
    static OBB obb_from_geometry(const ElementGeometry& geo);
};

std::ostream& operator<<(std::ostream& os, const Element& e);
std::ostream& operator<<(std::ostream& os, const ElementFeature& f);
} // namespace session_cpp
