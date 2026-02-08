#include "cylinder.h"
#include "cylinder.pb.h"
#include "line.pb.h"
#include "mesh.pb.h"
#include "xform.pb.h"
#include <cmath>
#include <fstream>

namespace session_cpp {

Cylinder::Cylinder(const Line& line, double radius) 
    : radius(radius), line(line), mesh(create_cylinder_mesh(line, radius)) {
}

std::pair<std::vector<Point>, std::vector<std::array<size_t, 3>>> Cylinder::unit_cylinder_geometry() {
    std::vector<Point> vertices = {
        Point(0.5, 0.0, -0.5),
        Point(0.404508, 0.293893, -0.5),
        Point(0.154508, 0.475528, -0.5),
        Point(-0.154508, 0.475528, -0.5),
        Point(-0.404508, 0.293893, -0.5),
        Point(-0.5, 0.0, -0.5),
        Point(-0.404508, -0.293893, -0.5),
        Point(-0.154508, -0.475528, -0.5),
        Point(0.154508, -0.475528, -0.5),
        Point(0.404508, -0.293893, -0.5),
        Point(0.5, 0.0, 0.5),
        Point(0.404508, 0.293893, 0.5),
        Point(0.154508, 0.475528, 0.5),
        Point(-0.154508, 0.475528, 0.5),
        Point(-0.404508, 0.293893, 0.5),
        Point(-0.5, 0.0, 0.5),
        Point(-0.404508, -0.293893, 0.5),
        Point(-0.154508, -0.475528, 0.5),
        Point(0.154508, -0.475528, 0.5),
        Point(0.404508, -0.293893, 0.5),
    };

    std::vector<std::array<size_t, 3>> triangles = {
        {0, 1, 11}, {0, 11, 10},
        {1, 2, 12}, {1, 12, 11},
        {2, 3, 13}, {2, 13, 12},
        {3, 4, 14}, {3, 14, 13},
        {4, 5, 15}, {4, 15, 14},
        {5, 6, 16}, {5, 16, 15},
        {6, 7, 17}, {6, 17, 16},
        {7, 8, 18}, {7, 18, 17},
        {8, 9, 19}, {8, 19, 18},
        {9, 0, 10}, {9, 10, 19},
    };

    return {vertices, triangles};
}

Xform Cylinder::line_to_cylinder_transform(const Line& line, double radius) {
    Point start = line.start();
    Point end = line.end();
    Vector line_vec = line.to_vector();
    double length = line.length();

    Vector z_axis = line_vec;
    z_axis.normalize_self();
    
    Vector x_axis;
    if (std::abs(z_axis[2]) < 0.9) {
        x_axis = Vector(0.0, 0.0, 1.0).cross(z_axis);
        x_axis.normalize_self();
    } else {
        x_axis = Vector(1.0, 0.0, 0.0).cross(z_axis);
        x_axis.normalize_self();
    }
    
    Vector y_axis = z_axis.cross(x_axis);
    y_axis.normalize_self();

    Xform scale = Xform::scale_xyz(radius * 2.0, radius * 2.0, length);
    
    // Create rotation matrix from column vectors
    Xform rotation;
    rotation.m[0] = x_axis[0];
    rotation.m[1] = x_axis[1];
    rotation.m[2] = x_axis[2];
    rotation.m[4] = y_axis[0];
    rotation.m[5] = y_axis[1];
    rotation.m[6] = y_axis[2];
    rotation.m[8] = z_axis[0];
    rotation.m[9] = z_axis[1];
    rotation.m[10] = z_axis[2];
    
    Point center(
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5
    );
    Xform translation = Xform::translation(center[0], center[1], center[2]);

    return translation * rotation * scale;
}

Mesh Cylinder::transform_geometry(
    const std::pair<std::vector<Point>, std::vector<std::array<size_t, 3>>>& geometry,
    const Xform& xform
) {
    const auto& [vertices, triangles] = geometry;
    Mesh mesh;

    std::vector<size_t> vertex_keys;
    vertex_keys.reserve(vertices.size());
    for (const auto& v : vertices) {
        Point transformed = xform.transformed_point(v);
        vertex_keys.push_back(mesh.add_vertex(transformed));
    }

    for (const auto& tri : triangles) {
        std::vector<size_t> face_vertices = {
            vertex_keys[tri[0]],
            vertex_keys[tri[1]],
            vertex_keys[tri[2]]
        };
        mesh.add_face(face_vertices);
    }

    return mesh;
}

Mesh Cylinder::create_cylinder_mesh(const Line& line, double radius) {
    auto unit_cylinder = unit_cylinder_geometry();
    Xform xform = line_to_cylinder_transform(line, radius);
    return transform_geometry(unit_cylinder, xform);
}

void Cylinder::transform() {
  line.transform();
  xform = Xform::identity();
}

Cylinder Cylinder::transformed() const {
  Cylinder result = *this;
  result.transform();
  return result;
}

nlohmann::ordered_json Cylinder::jsondump() const {
    nlohmann::ordered_json data;
    data["type"] = "Cylinder";
    data["guid"] = guid;
    data["name"] = name;
    data["radius"] = radius;
    data["line"] = line.jsondump();
    data["mesh"] = mesh.jsondump();
    return data;
}

Cylinder Cylinder::jsonload(const nlohmann::json& data) {
    Line line = Line::jsonload(data["line"]);
    double radius = data["radius"];
    Cylinder cylinder(line, radius);
    
    if (data.contains("guid")) {
        cylinder.guid = data["guid"];
    }
    if (data.contains("name")) {
        cylinder.name = data["name"];
    }
    if (data.contains("xform")) {
        cylinder.xform = Xform::jsonload(data["xform"]);
    }
    
    return cylinder;
}

std::string Cylinder::json_dumps() const {
    return jsondump().dump();
}

Cylinder Cylinder::json_loads(const std::string& json_string) {
    return jsonload(nlohmann::json::parse(json_string));
}

void Cylinder::json_dump(const std::string& filename) const {
    std::ofstream file(filename);
    file << jsondump().dump(4);
}

Cylinder Cylinder::json_load(const std::string& filename) {
    std::ifstream file(filename);
    nlohmann::json data;
    file >> data;
    return jsonload(data);
}

std::string Cylinder::pb_dumps() const {
    session_proto::Cylinder proto;
    proto.set_guid(guid);
    proto.set_name(name);
    proto.set_radius(radius);
    proto.mutable_line()->ParseFromString(line.pb_dumps());
    proto.mutable_mesh()->ParseFromString(mesh.pb_dumps());
    auto* xform_proto = proto.mutable_xform();
    xform_proto->set_guid(xform.guid);
    xform_proto->set_name(xform.name);
    for (int i = 0; i < 16; ++i) {
        xform_proto->add_matrix(xform.m[i]);
    }
    return proto.SerializeAsString();
}

Cylinder Cylinder::pb_loads(const std::string& data) {
    session_proto::Cylinder proto;
    proto.ParseFromString(data);
    Line ln = Line::pb_loads(proto.line().SerializeAsString());
    Cylinder cyl(ln, proto.radius());
    cyl.guid = proto.guid();
    cyl.name = proto.name();
    if (proto.has_mesh()) {
        cyl.mesh = Mesh::pb_loads(proto.mesh().SerializeAsString());
    }
    if (proto.has_xform()) {
        cyl.xform = Xform::pb_loads(proto.xform().SerializeAsString());
    }
    return cyl;
}

void Cylinder::pb_dump(const std::string& filename) const {
    std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

Cylinder Cylinder::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
    return pb_loads(data);
}

} // namespace session_cpp
