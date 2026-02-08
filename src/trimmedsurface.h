#pragma once

#include "nurbssurface.h"
#include "nurbscurve.h"
#include "xform.h"
#include "color.h"
#include "mesh.h"
#include "guid.h"
#include "json.h"
#include <vector>
#include <string>

namespace session_cpp {

class TrimmedSurface {
public:
    std::string guid = ::guid();
    std::string name = "my_trimmedsurface";
    double width = 1.0;
    Color surfacecolor = Color::black();
    Xform xform = Xform::identity();

    NurbsSurface m_surface;
    NurbsCurve m_outer_loop;
    std::vector<NurbsCurve> m_inner_loops;

public:
    // Static Factory Methods
    static TrimmedSurface create(const NurbsSurface& surface, const NurbsCurve& outer_loop);
    static TrimmedSurface create_planar(const NurbsCurve& boundary);

    // Constructors
    TrimmedSurface();
    TrimmedSurface(const TrimmedSurface& other);
    TrimmedSurface& operator=(const TrimmedSurface& other);
    bool operator==(const TrimmedSurface& other) const;
    bool operator!=(const TrimmedSurface& other) const;
    ~TrimmedSurface();

    // Accessors
    NurbsSurface surface() const;
    NurbsCurve get_outer_loop() const;
    void set_outer_loop(const NurbsCurve& loop);
    bool is_trimmed() const;
    bool is_valid() const;

    // Inner Loops
    void add_inner_loop(const NurbsCurve& loop_2d);
    void add_hole(const NurbsCurve& curve_3d);
    void add_holes(const std::vector<NurbsCurve>& curves_3d);
    NurbsCurve get_inner_loop(int index) const;
    int inner_loop_count() const;
    void clear_inner_loops();

    // Evaluation (delegates to m_surface)
    Point point_at(double u, double v) const;
    Vector normal_at(double u, double v) const;

    // Meshing
    Mesh mesh() const;

    // Transformation
    void transform();
    TrimmedSurface transformed() const;

    // JSON Serialization
    nlohmann::ordered_json jsondump() const;
    static TrimmedSurface jsonload(const nlohmann::json& data);
    void json_dump(const std::string& filename) const;
    static TrimmedSurface json_load(const std::string& filename);
    std::string json_dumps() const;
    static TrimmedSurface json_loads(const std::string& json_string);

    // Protobuf Serialization
    std::string pb_dumps() const;
    static TrimmedSurface pb_loads(const std::string& data);
    void pb_dump(const std::string& filename) const;
    static TrimmedSurface pb_load(const std::string& filename);

    // String Representation
    std::string str() const;
    std::string repr() const;
    friend std::ostream& operator<<(std::ostream& os, const TrimmedSurface& ts);

private:
    void deep_copy_from(const TrimmedSurface& src);
};

} // namespace session_cpp
