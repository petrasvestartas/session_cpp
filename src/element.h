#pragma once
#include "guid.h"
#include "json.h"
#include "mesh.h"
#include "brep.h"
#include "obb.h"
#include "xform.h"
#include "line.h"
#include "point.h"
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
    std::string guid = ::guid();
    std::string name;
    Xform session_transformation = Xform::identity();

    Element(const std::string& name = "my_element");
    Element(const Mesh& geometry, const std::string& name = "my_element");
    Element(const BRep& geometry, const std::string& name = "my_element");
    Element(const Element& other);
    Element& operator=(const Element& other);
    virtual ~Element() = default;

    const ElementGeometry& geometry() const { return _geometry; }
    bool has_geometry() const;
    std::string geometry_type_name() const;
    ElementGeometry session_geometry() const;
    Obb aabb();
    Obb obb();
    Mesh collision_mesh();
    Point point();
    bool is_dirty() const { return _is_dirty; }
    const std::optional<Obb>& cached_aabb() const { return _aabb; }
    const std::optional<Obb>& cached_obb() const { return _obb; }
    const std::optional<Mesh>& cached_collision_mesh() const { return _collision_mesh; }
    const std::optional<Point>& cached_point() const { return _point; }
    size_t features_count() const { return _features.size(); }

    void add_feature(std::function<Mesh(Mesh)> f);
    void set_geometry(const Mesh& geo);
    void set_geometry(const BRep& geo);
    void reset();

    Element duplicate() const;
    virtual bool operator==(const Element& other) const;
    bool operator!=(const Element& other) const;
    virtual std::string str() const;
    virtual std::string repr() const;
    friend std::ostream& operator<<(std::ostream& os, const Element& e);

    virtual nlohmann::ordered_json jsondump() const;
    static Element jsonload(const nlohmann::json& data);
    std::string json_dumps() const;
    static Element json_loads(const std::string& s);
    void json_dump(const std::string& path) const;
    static Element json_load(const std::string& path);

    virtual std::string pb_dumps() const;
    static Element pb_loads(const std::string& data);
    void pb_dump(const std::string& path) const;
    static Element pb_load(const std::string& path);

protected:
    ElementGeometry _geometry;
    bool _is_dirty = true;
    std::optional<Obb> _aabb;
    std::optional<Obb> _obb;
    std::optional<Mesh> _collision_mesh;
    std::optional<Point> _point;
    std::vector<std::function<Mesh(Mesh)>> _features;

    Obb compute_aabb();
    Obb compute_obb();
    Mesh compute_collision_mesh();
    Point compute_point();
    Mesh apply_features(Mesh geo) const;
    static Obb obb_from_geometry(const ElementGeometry& geo, bool as_aabb);
};

class ColumnElement : public Element {
public:
    ColumnElement(double width = 0.4, double depth = 0.4, double height = 3.0,
                  const std::string& name = "my_column");
    ColumnElement(const ColumnElement& other);
    ColumnElement& operator=(const ColumnElement& other);

    double width() const { return _width; }
    double depth() const { return _depth; }
    double height() const { return _height; }
    void set_width(double v);
    void set_depth(double v);
    void set_height(double v);
    Line center_line() const;
    void extend(double distance);
    Mesh compute_element_geometry() const;

    ColumnElement duplicate() const;
    bool operator==(const Element& other) const override;
    std::string str() const override;
    std::string repr() const override;
    nlohmann::ordered_json jsondump() const override;
    std::string pb_dumps() const override;

    static ColumnElement jsonload(const nlohmann::json& data);
    static ColumnElement json_loads(const std::string& s);
    static ColumnElement json_load(const std::string& path);
    static ColumnElement pb_loads(const std::string& data);
    static ColumnElement pb_load(const std::string& path);

private:
    double _width, _depth, _height;
};

class BeamElement : public Element {
public:
    BeamElement(double width = 0.1, double depth = 0.2, double length = 3.0,
                const std::string& name = "my_beam");
    BeamElement(const BeamElement& other);
    BeamElement& operator=(const BeamElement& other);

    double width() const { return _width; }
    double depth() const { return _depth; }
    double length() const { return _length; }
    void set_width(double v);
    void set_depth(double v);
    void set_length(double v);
    Line center_line() const;
    void extend(double distance);
    Mesh compute_element_geometry() const;

    BeamElement duplicate() const;
    bool operator==(const Element& other) const override;
    std::string str() const override;
    std::string repr() const override;
    nlohmann::ordered_json jsondump() const override;
    std::string pb_dumps() const override;

    static BeamElement jsonload(const nlohmann::json& data);
    static BeamElement json_loads(const std::string& s);
    static BeamElement json_load(const std::string& path);
    static BeamElement pb_loads(const std::string& data);
    static BeamElement pb_load(const std::string& path);

private:
    double _width, _depth, _length;
};

class PlateElement : public Element {
public:
    PlateElement(const std::vector<Point>& polygon = {}, double thickness = 0.1,
                 const std::string& name = "my_plate");
    PlateElement(const PlateElement& other);
    PlateElement& operator=(const PlateElement& other);

    const std::vector<Point>& polygon() const { return _polygon; }
    double thickness() const { return _thickness; }
    void set_polygon(const std::vector<Point>& pts);
    void set_thickness(double v);
    Mesh compute_element_geometry() const;
    static Vector polygon_normal(const std::vector<Point>& pts);

    PlateElement duplicate() const;
    bool operator==(const Element& other) const override;
    std::string str() const override;
    std::string repr() const override;
    nlohmann::ordered_json jsondump() const override;
    std::string pb_dumps() const override;

    static PlateElement jsonload(const nlohmann::json& data);
    static PlateElement json_loads(const std::string& s);
    static PlateElement json_load(const std::string& path);
    static PlateElement pb_loads(const std::string& data);
    static PlateElement pb_load(const std::string& path);

private:
    std::vector<Point> _polygon;
    double _thickness;
};

std::ostream& operator<<(std::ostream& os, const Element& e);
} // namespace session_cpp
