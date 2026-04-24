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
    Xform session_transformation = Xform::identity();

    Element(const std::string& name = "my_element", const Xform& transformation = Xform::identity());
    Element(const Mesh& geometry, const std::string& name = "my_element", const Xform& transformation = Xform::identity());
    Element(const BRep& geometry, const std::string& name = "my_element", const Xform& transformation = Xform::identity());
    Element(const Element& other);
    Element& operator=(const Element& other);
    virtual ~Element() = default;

    const ElementGeometry& geometry() const { return _geometry; }
    bool has_geometry() const;
    std::string geometry_type_name() const;
    ElementGeometry session_geometry() const;
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

class ElementColumn : public Element {
public:
    ElementColumn(double width = 0.4, double depth = 0.4, double height = 3.0,
                  const std::string& name = "my_column", const Xform& transformation = Xform::identity());
    ElementColumn(const ElementColumn& other);
    ElementColumn& operator=(const ElementColumn& other);

    double width() const { return _width; }
    double depth() const { return _depth; }
    double height() const { return _height; }
    void set_width(double v);
    void set_depth(double v);
    void set_height(double v);
    Line center_line() const;
    void extend(double distance);
    Mesh compute_element_geometry() const;
    std::vector<Polyline> compute_polylines() const override;
    std::vector<Plane> compute_planes() const override;
    std::vector<Vector> compute_edge_vectors() const override;
    std::optional<Line> compute_axis() const override;

    ElementColumn duplicate() const;
    bool operator==(const Element& other) const override;
    std::string str() const override;
    std::string repr() const override;
    nlohmann::ordered_json jsondump() const override;
    std::string pb_dumps() const override;

    static ElementColumn jsonload(const nlohmann::json& data);
    static ElementColumn file_json_loads(const std::string& s);
    static ElementColumn file_json_load(const std::string& path);
    static ElementColumn pb_loads(const std::string& data);
    static ElementColumn pb_load(const std::string& path);

private:
    double _width, _depth, _height;
};

class ElementBeam : public Element {
public:
    ElementBeam(double width = 0.1, double depth = 0.2, double length = 3.0,
                const std::string& name = "my_beam", const Xform& transformation = Xform::identity());
    ElementBeam(const ElementBeam& other);
    ElementBeam& operator=(const ElementBeam& other);

    double width() const { return _width; }
    double depth() const { return _depth; }
    double length() const { return _length; }
    void set_width(double v);
    void set_depth(double v);
    void set_length(double v);
    Line center_line() const;
    void extend(double distance);
    Mesh compute_element_geometry() const;
    std::vector<Polyline> compute_polylines() const override;
    std::vector<Plane> compute_planes() const override;
    std::vector<Vector> compute_edge_vectors() const override;
    std::optional<Line> compute_axis() const override;

    ElementBeam duplicate() const;
    bool operator==(const Element& other) const override;
    std::string str() const override;
    std::string repr() const override;
    nlohmann::ordered_json jsondump() const override;
    std::string pb_dumps() const override;

    static ElementBeam jsonload(const nlohmann::json& data);
    static ElementBeam file_json_loads(const std::string& s);
    static ElementBeam file_json_load(const std::string& path);
    static ElementBeam pb_loads(const std::string& data);
    static ElementBeam pb_load(const std::string& path);

private:
    double _width, _depth, _length;
};

struct JointConnection {
    int joint_id;
    bool is_male;
    double parameter;
};

class ElementPlate : public Element {
public:
    ElementPlate(const std::vector<Point>& polygon = {}, double thickness = 0.1,
                 const std::string& name = "my_plate", const Xform& transformation = Xform::identity());
    ElementPlate(const std::vector<Point>& bottom, const std::vector<Point>& top,
                 const std::string& name = "my_plate", const Xform& transformation = Xform::identity());
    ElementPlate(const Polyline& bottom, const Polyline& top,
                 const std::string& name = "my_plate", const Xform& transformation = Xform::identity());
    ElementPlate(const ElementPlate& other);
    ElementPlate& operator=(const ElementPlate& other);

    const std::vector<Point>& polygon() const { return _polygon; }
    const std::vector<Point>& polygon_top() const { return _polygon_top; }
    double thickness() const { return _thickness; }
    void set_polygon(const std::vector<Point>& pts);
    void set_polygon_top(const std::vector<Point>& pts);
    void set_thickness(double v);
    Mesh compute_element_geometry() const;
    AABB compute_aabb_fast(double inflate = 0.0) const;
    static Vector polygon_normal(const std::vector<Point>& pts);

    const std::vector<int>& joint_types() const { return _joint_types; }
    void set_joint_types(const std::vector<int>& v) { _joint_types = v; }
    const std::vector<std::vector<JointConnection>>& j_mf() const { return _j_mf; }
    void set_j_mf(const std::vector<std::vector<JointConnection>>& v) { _j_mf = v; }
    const std::string& key() const { return _key; }
    void set_key(const std::string& v) { _key = v; }
    const std::optional<Plane>& component_plane() const { return _component_plane; }
    void set_component_plane(const Plane& v) { _component_plane = v; }

    std::vector<Polyline> compute_polylines() const override;
    std::vector<Plane> compute_planes() const override;
    std::vector<Vector> compute_edge_vectors() const override;
    std::optional<Line> compute_axis() const override;

    ElementPlate duplicate() const;
    bool operator==(const Element& other) const override;
    std::string str() const override;
    std::string repr() const override;
    nlohmann::ordered_json jsondump() const override;
    std::string pb_dumps() const override;

    static ElementPlate jsonload(const nlohmann::json& data);
    static ElementPlate file_json_loads(const std::string& s);
    static ElementPlate file_json_load(const std::string& path);
    static ElementPlate pb_loads(const std::string& data);
    static ElementPlate pb_load(const std::string& path);

private:
    std::vector<Point> _polygon;
    std::vector<Point> _polygon_top;
    double _thickness;
    std::vector<int> _joint_types;
    std::vector<std::vector<JointConnection>> _j_mf;
    std::string _key;
    std::optional<Plane> _component_plane;
};

std::ostream& operator<<(std::ostream& os, const Element& e);
} // namespace session_cpp
