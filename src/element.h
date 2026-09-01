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
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace session_cpp {

using ElementGeometry = std::variant<std::monostate, Mesh, BRep>;

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
    size_t features_count() const { return _features.size(); }

    void add_feature(std::function<Mesh(Mesh)> f);
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
    std::vector<std::function<Mesh(Mesh)>> _features;

    OBB compute_aabb();
    OBB compute_obb();
    Mesh compute_collision_mesh();
    Point compute_point();
    virtual std::vector<Polyline> compute_polylines() const;
    virtual std::vector<Plane> compute_planes() const;
    virtual std::vector<Vector> compute_edge_vectors() const;
    virtual std::optional<Line> compute_axis() const;
    Mesh apply_features(Mesh geo) const;
    static OBB obb_from_geometry(const ElementGeometry& geo);
};


std::ostream& operator<<(std::ostream& os, const Element& e);
} // namespace session_cpp
