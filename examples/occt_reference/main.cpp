#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepLib.hxx>
#include <BRepOffsetAPI_MakeOffsetShape.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <json.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

using nlohmann::json;

static json read(const std::filesystem::path& path) {

    std::ifstream stream(path);
    return json::parse(stream);
}

static gp_Pnt point(const json& values) {

    return {values[0].get<double>(), values[1].get<double>(), values[2].get<double>()};
}

static TopoDS_Solid solid(const json& data) {

    BRepBuilderAPI_Sewing sewing(1e-7);
    for (const json& use : data.at("shells")[0].at("faces")) {
        const json& face = data.at("faces")[use.at("index").get<int>()];
        const json& wire = data.at("wires")[face.at("wires")[0].at("index").get<int>()];
        BRepBuilderAPI_MakePolygon polygon;
        for (const json& reference : wire.at("edges")) {
            const json& edge = data.at("edges")[reference.at("index").get<int>()];
            const bool reversed = reference.at("orientation") == "reversed";
            const int vertex = edge.at(reversed ? "end_vertex" : "start_vertex").get<int>();
            polygon.Add(point(data.at("vertices")[vertex].at("point")));
        }
        polygon.Close();
        TopoDS_Face patch = BRepBuilderAPI_MakeFace(polygon.Wire()).Face();
        if (use.at("orientation") == "reversed")
            patch.Reverse();
        sewing.Add(patch);
    }
    sewing.Perform();
    TopoDS_Solid result = BRepBuilderAPI_MakeSolid(TopoDS::Shell(sewing.SewedShape())).Solid();
    BRepLib::OrientClosedSolid(result);
    return result;
}

static gp_Pnt mesh_point(const json& vertices, size_t index) {

    const json& vertex = vertices.at(std::to_string(index));
    return {vertex.at("x").get<double>(), vertex.at("y").get<double>(), vertex.at("z").get<double>()};
}

static double mesh_volume(const json& mesh) {

    const json& vertices = mesh.at("vertex");
    const gp_Pnt origin = mesh_point(vertices, std::stoull(vertices.begin().key()));
    double volume = 0.0;
    for (const json& face : mesh.at("face")) {
        const gp_Vec a(origin, mesh_point(vertices, face[0].get<size_t>()));
        for (size_t i = 1; i + 1 < face.size(); ++i) {
            const gp_Vec b(origin, mesh_point(vertices, face[i].get<size_t>()));
            const gp_Vec c(origin, mesh_point(vertices, face[i + 1].get<size_t>()));
            volume += a.Dot(b.Crossed(c)) / 6.0;
        }
    }
    return std::abs(volume);
}

static double radius(const json& data) {

    const json& face = data.at("faces")[0];
    const json& wire = data.at("wires")[face.at("wires")[0].at("index").get<int>()];
    std::vector<gp_Pnt> points;
    for (const json& reference : wire.at("edges")) {
        const json& edge = data.at("edges")[reference.at("index").get<int>()];
        const bool reversed = reference.at("orientation") == "reversed";
        const int index = edge.at(reversed ? "end_vertex" : "start_vertex").get<int>();
        points.push_back(point(data.at("vertices")[index].at("point")));
    }
    const gp_Vec a(points[0], points[1]);
    const gp_Vec b(points[0], points[2]);
    const gp_Vec normal = a.Crossed(b).Normalized();
    return std::abs(normal.Dot(gp_Vec(gp_Pnt(), points[0])));
}

int main() {

    const std::filesystem::path folder = GEOMETRY_FIXTURES;
    json report = json::array();
    int failures = 0;
    for (const std::string name : {"box", "tetrahedron", "cube", "octahedron", "dodecahedron", "icosahedron", "bevel"}) {
        const std::string prefix = "offset_" + name;
        const json data = read(folder / (prefix + "_original.brep.json"));
        const TopoDS_Solid shape = solid(data);
        for (const std::string direction : {"outward", "inward"}) {
            if (name == "bevel" && direction == "outward")
                continue;

            double distance = name == "box" ? 0.25 : radius(data) * 0.2;
            if (name == "bevel")
                distance = 0.5;
            if (direction == "inward")
                distance = -distance;

            json result = {
                {"case", prefix + "_" + direction},
                {"distance", distance},
                {"occt_input_valid", BRepCheck_Analyzer(shape).IsValid()}
            };
            try {
                BRepOffsetAPI_MakeOffsetShape operation;
                operation.PerformByJoin(
                    shape,
                    distance,
                    1e-7,
                    BRepOffset_Skin,
                    false,
                    false,
                    GeomAbs_Intersection,
                    true
                );
                const bool done = operation.IsDone();
                result["occt_done"] = done;
                bool valid = false;
                double reference = 0.0;
                if (done) {
                    const TopoDS_Shape offset = operation.Shape();
                    valid = BRepCheck_Analyzer(offset).IsValid();
                    GProp_GProps properties;
                    BRepGProp::VolumeProperties(offset, properties);
                    reference = std::abs(properties.Mass());
                }
                const double native = mesh_volume(read(folder / (prefix + "_" + direction + ".mesh.json")));
                const double error = std::abs(native - reference) / std::max(1.0, native);
                const bool passed = done && valid && error < 1e-8;
                result["native_volume"] = native;
                result["occt_volume"] = reference;
                result["occt_valid"] = valid;
                result["relative_error"] = error;
                result["passed"] = passed;
                if (!passed)
                    ++failures;
            } catch (const Standard_Failure& error) {
                result["passed"] = false;
                result["error"] = error.GetMessageString();
                ++failures;
            }
            report.push_back(result);
        }
    }
    std::ofstream output(folder / "occt_reference.json");
    output << report.dump(2);
    std::cout << report.size() - failures << "/" << report.size() << " OCCT offset comparisons passed\n";
    return failures;
}
