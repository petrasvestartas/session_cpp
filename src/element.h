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

/// One modification applied to a host element - a cut, a drill, a joint pocket.
///
/// The serializable half of what `Element::add_geometry_op` cannot be: that takes a
/// `std::function<Mesh(Mesh)>`, so an operation applied in memory vanishes the moment the
/// Session is written. Domains worked around it by adding flat arrays to Element - a joint
/// type code per face - which is how timber fields ended up in element.proto and had to be
/// reserved out again. An ElementFeature carries the same information in a shape every domain
/// can use.
///
/// The kernel does not know how to APPLY one: `feature_type` means something only to the
/// package that wrote it. It knows enough to DRAW one, which is what lets a viewer show
/// features from a package it has never heard of.
struct ElementFeature {
    std::string name;               ///< Human-readable label
    std::string feature_type;       ///< e.g. "cut", "drill", "joint"; the package's vocabulary
    int face_index = -1;            ///< Face of the host this applies to; -1 = whole element
    std::vector<Polyline> outlines; ///< Geometry of the modification

    ElementFeature() = default;
    ElementFeature(std::string feature_type, int face_index, std::vector<Polyline> outlines,
                   std::string name = "")
        : name(std::move(name)), feature_type(std::move(feature_type)),
          face_index(face_index), outlines(std::move(outlines)) {}

    /// Lazily minted, like every other identity in the kernel - a feature nobody names never
    /// pays for a guid.
    ///
    /// A feature is addressable in its own right: the package that wrote a joint needs to name
    /// it again later, to update it, to report a clash against it, or to let a viewer select one
    /// of the forty cuts on a beam. The only other handle is the index in `features`, and that
    /// moves the moment an earlier feature is removed.
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
    /// Clear the guid so a FRESH one mints on next read - the duplicate enabler.
    void refresh_guid() { _guid.clear(); }

    /// A copy is a new feature that happens to look the same, so it gets a new identity; a move
    /// is the same feature in a new place, so the identity travels.
    ///
    /// Declaring the move members is not optional once a guid exists. A user-declared copy
    /// constructor suppresses the implicit move, so a `pb_loads` that misses NRVO falls back to
    /// the COPY and silently drops the guid it has just decoded - every other field survives, so
    /// the object looks right and only its identity is wrong. That is what broke Line on MSVC;
    /// see line.h.
    ElementFeature(const ElementFeature& other)
        : name(other.name), feature_type(other.feature_type),
          face_index(other.face_index), outlines(other.outlines) {}
    ElementFeature& operator=(const ElementFeature& other) {
        if (this != &other) {
            _guid.clear();
            name = other.name;
            feature_type = other.feature_type;
            face_index = other.face_index;
            outlines = other.outlines;
        }
        return *this;
    }
    ElementFeature(ElementFeature&& other) noexcept = default;
    ElementFeature& operator=(ElementFeature&& other) noexcept = default;

    /// Data equality, not identity: two features describing the same cut on the same face ARE
    /// equal, exactly as `Line::operator==` ignores its guid.
    bool operator==(const ElementFeature& other) const;
    bool operator!=(const ElementFeature& other) const { return !(*this == other); }
    std::string str() const;
    std::string repr() const;
    friend std::ostream& operator<<(std::ostream& os, const ElementFeature& f);

    /// A feature serializes on its own, not only as part of its host. A package that stores a
    /// library of standard cuts, or reports one across a wire, has a single feature in hand and
    /// nothing to attach it to - and every other class in the kernel round-trips by itself.
    nlohmann::ordered_json jsondump() const;
    static ElementFeature jsonload(const nlohmann::json& data);
    void file_json_dump(const std::string& filename) const;
    static ElementFeature file_json_load(const std::string& filename);
    std::string file_json_dumps() const;
    static ElementFeature file_json_loads(const std::string& json_string);

    std::string pb_dumps() const;
    static ElementFeature pb_loads(const std::string& data);
    void pb_dump(const std::string& filename) const;
    static ElementFeature pb_load(const std::string& filename);

private:
    mutable std::string _guid;
};

class Element {
public:
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string name;

    Element(const std::string& name = "my_element");
    Element(const Mesh& geometry, const std::string& name = "my_element");
    Element(const BRep& geometry, const std::string& name = "my_element");
    Element(const Element& other);
    /// Move constructor and assignment: identity SURVIVES a move.
    ///
    /// A copy is a new object and mints a new guid; a move is the SAME object in a new place, so
    /// `_guid` transfers. Declaring these is also what keeps `return x;` safe: the copy below is
    /// user-declared, which suppresses the implicit move, so a `pb_loads` that missed NRVO fell back
    /// to the COPY and silently dropped the guid it had just deserialized - every other field
    /// survived, so the object looked right and only its identity was wrong. That is what broke Line
    /// on MSVC when its pb_loads changed shape; see line.h.
    Element(Element&& other) noexcept = default;
    Element& operator=(Element&& other) noexcept = default;

    Element& operator=(const Element& other);
    virtual ~Element() = default;

    const ElementGeometry& geometry() const { return _geometry; }
    bool has_geometry() const;
    std::string geometry_type_name() const;
    /// The element's geometry placed by `xform`. The placement is supplied by the caller -
    /// an Element no longer stores one; the Session does. Pass identity for local geometry.
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
    /// In-memory mesh operations, applied lazily when geometry is computed.
    ///
    /// Renamed off "feature": these are `std::function`s and CANNOT be serialized, whereas
    /// `features()` below is data that survives a Session round trip. Two different things
    /// wearing one name was the confusion that made a joint type code look like it needed its
    /// own field on Element.
    size_t geometry_ops_count() const { return _geometry_ops.size(); }
    void add_geometry_op(std::function<Mesh(Mesh)> f);

    /// Modifications carried BY this element, and written with it.
    const std::vector<ElementFeature>& features() const { return _features; }
    void set_features(std::vector<ElementFeature> features) { _features = std::move(features); }
    void add_feature(ElementFeature feature) { _features.push_back(std::move(feature)); }
    size_t features_count() const { return _features.size(); }

    /// Direction(s) the element is inserted along when the assembly is put together. General to
    /// any assembly: it is what an assembly sequence is ordered by. Plural because an element
    /// with several jointed faces can admit a different direction per face.
    const std::vector<Vector>& insertion_vectors() const { return _insertion_vectors; }
    void set_insertion_vectors(std::vector<Vector> v) { _insertion_vectors = std::move(v); }

    /// NOMINAL extents in this element's own frame - authored design intent, NOT a measurement.
    /// Plate: x/y outline extent, z thickness. Beam: x/y cross-section, z length.
    ///
    /// Deliberately distinct from obb(), which MEASURES the geometry that exists. The two are
    /// allowed to disagree: a thickness drives a loft before there is any geometry to measure,
    /// so the nominal value has to exist first and outlive what is built from it. Read obb()
    /// for how big it IS, this for how big it was MEANT to be. Nullopt = never authored.
    const std::optional<Vector>& dimensions() const { return _dimensions; }
    void set_dimensions(const Vector& d) { _dimensions = d; }
    /// Bake a placement into this element's own geometry, invalidating the cached boxes.
    /// The Session owns the placement, so it hands it in here rather than the Element storing it.
    void place(const Xform& xform);
    void set_geometry(const Mesh& geo);
    void set_geometry(const BRep& geo);
    void set_polylines(std::vector<Polyline> polys);
    void set_planes(std::vector<Plane> plns);
    void reset();

    Element duplicate() const;
    virtual bool operator==(const Element& other) const;
    bool operator!=(const Element& other) const;
    virtual std::string str() const;
    virtual std::string repr() const;
    friend std::ostream& operator<<(std::ostream& os, const Element& e);

    virtual nlohmann::ordered_json jsondump() const;
    static Element jsonload(const nlohmann::json& data);
    std::string file_json_dumps() const;
    static Element file_json_loads(const std::string& s);
    void file_json_dump(const std::string& path) const;
    static Element file_json_load(const std::string& path);

    virtual std::string pb_dumps() const;
    static Element pb_loads(const std::string& data);

    // ── Polymorphic elements ────────────────────────────────────────────────────────────
    // An Element is a geometry container and knows nothing about domains (see element.proto).
    // A downstream package that needs more - wood's plate carries insertion vectors, joint
    // type codes, cut outlines - registers a factory under its own type name. The name goes
    // in `element_type` and the extra state in `element_data`; the kernel copies both through
    // without interpreting either.
    //
    // Why a registry rather than more virtuals: `pb_loads` is a static returning by VALUE, so
    // a derived element loaded through Objects was sliced to its base. Dumping already
    // dispatched (`pb_dumps` is virtual), so a subclass wrote its payload correctly and then
    // lost it on the way back in. The factory is the missing half of that round trip.

    /// This element's own type name, written to `element_type`. A plain Element authored in
    /// memory returns "" - proto3 omits empty strings, so its bytes are byte-identical to
    /// before this existed, which is what keeps the cross-language golden files valid.
    ///
    /// A base Element that was LOADED from a derived element's bytes returns the type it was
    /// carrying. Without that, opening a wood file in a viewer that has no wood registered and
    /// saving it again wrote `element_type=""` and destroyed the payload - the kernel promised
    /// above to copy both through untouched, and this is the half that keeps the promise. A
    /// derived class overrides this and never consults the carried value.
    virtual std::string element_type_name() const { return _element_type; }

    /// This element's own state, written to `element_data`. Opaque to the kernel; the format
    /// is entirely the registering package's business. Carried through for the same reason as
    /// `element_type_name` above.
    virtual std::string element_data_dumps() const { return _element_data; }

    /// Builds one element from a full serialized `session_proto.Element` - the same bytes
    /// `pb_loads` takes, so a factory can read the base fields as well as `element_data`.
    using Factory = std::function<std::shared_ptr<Element>(const std::string& data)>;

    /// Register `factory` for `type_name`. Idempotent re-registration of the same name
    /// replaces the previous factory. Call it once, before loading anything.
    static void register_type(const std::string& type_name, Factory factory);
    static bool is_registered(const std::string& type_name);
    static std::vector<std::string> registered_types();

    /// Load an element, preserving its derived type when one is registered. Falls back to a
    /// base Element when `element_type` is empty OR names a type nobody registered - an
    /// unknown domain type degrades to its geometry rather than failing the whole Session,
    /// which is what lets a viewer open a file written by a package it does not have.
    static std::shared_ptr<Element> pb_loads_polymorphic(const std::string& data);
    void pb_dump(const std::string& path) const;
    static Element pb_load(const std::string& path);

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
    /// The derived type name and payload this element was LOADED with, for an element whose
    /// type nobody registered. Empty on anything authored in memory. See the two virtuals.
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
    static OBB obb_from_geometry(const ElementGeometry& geo);
};


std::ostream& operator<<(std::ostream& os, const Element& e);
std::ostream& operator<<(std::ostream& os, const ElementFeature& f);
} // namespace session_cpp
