#pragma once

#include "line.h"
#include "mesh.h"
#include "point.h"
#include "vector.h"
#include "xform.h"
#include "guid.h"
#include "json.h"
#include <string>
#include <vector>
#include <array>

namespace session_cpp {

/**
 * @class Cylinder
 * @brief A cylinder geometry defined by a line and radius.
 * 
 * The cylinder is generated as a 10-sided cylinder mesh that is oriented
 * along the line direction and scaled to match the line length and specified radius.
 */
class Cylinder {
public:
    std::string guid = ::guid();
    std::string name = "my_cylinder";
    double radius;
    Line line;
    Mesh mesh;

    Xform xform;

    /**
     * @brief Creates a new Cylinder from a line and radius.
     * @param line The centerline of the cylinder
     * @param radius The radius of the cylinder
     * @return A new Cylinder with a generated 10-sided cylinder mesh
     */
    Cylinder(const Line& line, double radius);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    void transform();
    Cylinder transformed() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serializes the Cylinder to a JSON string
    nlohmann::ordered_json jsondump() const;
    
    /// Deserializes a Cylinder from JSON data
    static Cylinder jsonload(const nlohmann::json& data);
    
    std::string json_dumps() const;
    static Cylinder json_loads(const std::string& json_string);
    void json_dump(const std::string& filename) const;
    static Cylinder json_load(const std::string& filename);
    std::string pb_dumps() const;
    static Cylinder pb_loads(const std::string& data);
    void pb_dump(const std::string& filename) const;
    static Cylinder pb_load(const std::string& filename);

private:
    static Mesh create_cylinder_mesh(const Line& line, double radius);
    static std::pair<std::vector<Point>, std::vector<std::array<size_t, 3>>> unit_cylinder_geometry();
    static Xform line_to_cylinder_transform(const Line& line, double radius);
    static Mesh transform_geometry(const std::pair<std::vector<Point>, std::vector<std::array<size_t, 3>>>& geometry, const Xform& xform);
};

} // namespace session_cpp
