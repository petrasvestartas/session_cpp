#pragma once
#include "mesh.h"
#include <array>
#include <string>

namespace session_cpp::mesh_parametric_metadata {
inline thread_local unsigned capture_depth = 0;
struct Capture {
    Capture() { ++capture_depth; }
    ~Capture() { --capture_depth; }
    Capture(const Capture&) = delete;
    Capture& operator=(const Capture&) = delete;
};
using UV = std::array<double, 2>;
inline std::string name(size_t vertex, int direction) {
    return "tessellation_uv/" + std::to_string(vertex) + (direction ? "/v" : "/u");
}
inline void corner(Mesh& mesh, size_t face, size_t vertex, const UV& uv) {
    if (!capture_depth)
        return;
    mesh.facedata[face][name(vertex, 0)] = uv[0];
    mesh.facedata[face][name(vertex, 1)] = uv[1];
}
inline bool read(const Mesh& mesh, size_t face, size_t vertex, UV& uv) {
    const auto data = mesh.facedata.find(face);
    if (data == mesh.facedata.end())
        return false;
    for (int d = 0; d < 2; ++d) {
        const auto found = data->second.find(name(vertex, d));
        if (found == data->second.end())
            return false;
        uv[d] = found->second;
    }
    return true;
}
inline void triangle(Mesh& mesh, size_t face, const std::array<size_t, 3>& vertices,
                     const std::array<UV, 3>& uv, int path = 0) {
    if (capture_depth && path)
        mesh.facedata[face]["tessellation_path"] = path;
    if (capture_depth)
        for (int i = 0; i < 3; ++i)
            corner(mesh, face, vertices[i], uv[i]);
}
inline void from_vertices(Mesh& mesh) {
    if (!capture_depth)
        return;
    for (const auto& face : mesh.face)
        for (size_t key : face.second) {
            const Attributes& attributes = mesh.vertex.at(key).attributes;
            UV existing;
            if (!read(mesh, face.first, key, existing) && attributes.count("u") && attributes.count("v"))
                corner(mesh, face.first, key, {attributes.at("u"), attributes.at("v")});
        }
}
inline void copied_corner(Mesh& mesh, size_t face, size_t source, size_t target) {
    UV uv;
    if (!capture_depth)
        return;
    if (read(mesh, face, source, uv))
        corner(mesh, face, target, uv);
    std::string origin = "tessellation_origin/" + std::to_string(source);
    for (const auto& attribute : mesh.vertex.at(source).attributes)
        if (attribute.first.starts_with("tessellation_origin/"))
            origin = attribute.first;
    mesh.vertex[source].attributes[origin] = 1;
    mesh.vertex[target].attributes[origin] = 1;
}
}
