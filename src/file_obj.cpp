#include "file_obj.h"
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace session_cpp {
namespace file_obj {

std::string write_file_obj_to_string(const Mesh& mesh) {

    const auto [vertices, faces] = mesh.to_vertices_and_faces();
    std::ostringstream out;
    out << std::setprecision(std::numeric_limits<double>::max_digits10);

    for (const Point& p : vertices)
        out << "v " << p[0] << " " << p[1] << " " << p[2] << "\n";

    for (const std::vector<size_t>& face : faces) {
        if (face.size() < 3)
            continue;

        out << "f";

        for (const size_t i : face)
            out << " " << i + 1;

        out << "\n";
    }

    return out.str();
}

void write_file_obj(const Mesh& mesh, const std::string& filepath) {

    std::ofstream out(filepath);

    if (!out.is_open())
        return;

    out << write_file_obj_to_string(mesh);
}

Mesh read_file_obj_from_str(const std::string& content) {

    std::istringstream in(content);
    std::string line;
    std::vector<Point> verts;
    std::vector<std::vector<size_t>> faces;

    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#')
            continue;

        if (line.rfind("v ", 0) == 0) {
            std::istringstream iss(line.substr(2));
            double x;
            double y;
            double z;

            if (iss >> x >> y >> z)
                verts.emplace_back(x, y, z);
        } else if (line.rfind("f ", 0) == 0) {
            std::istringstream iss(line.substr(2));
            std::string tok;
            std::vector<size_t> face;

            while (iss >> tok) {
                const long long idx = std::atoll(tok.substr(0, tok.find('/')).c_str());

                if (idx == 0)
                    continue;

                const long long vidx = idx > 0 ? idx - 1 : static_cast<long long>(verts.size()) + idx;
                face.push_back(static_cast<size_t>(vidx));
            }

            if (face.size() >= 3)
                faces.push_back(face);
        }
    }

    return Mesh::from_vertices_and_faces(verts, faces);
}

Mesh read_file_obj(const std::string& filepath) {

    std::ifstream in(filepath);
    std::stringstream buffer;
    buffer << in.rdbuf();

    return read_file_obj_from_str(buffer.str());
}

std::vector<Polyline> read_file_obj_polylines(const std::string& filepath) {

    std::ifstream in(filepath);
    std::string line;
    std::vector<Point> verts;
    std::vector<Polyline> polylines;
    std::vector<long long> curv;
    bool in_curv = false;

    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#')
            continue;

        if (line.rfind("v ", 0) == 0) {
            std::istringstream iss(line.substr(2));
            double x;
            double y;
            double z;

            if (iss >> x >> y >> z)
                verts.emplace_back(x, y, z);
        } else if (line.rfind("curv ", 0) == 0) {
            std::istringstream iss(line.substr(5));
            std::string u0;
            std::string u1;
            iss >> u0 >> u1;
            curv.clear();
            long long idx;

            while (iss >> idx)
                curv.push_back(idx);

            in_curv = true;
        } else if (line.rfind("end", 0) == 0 && in_curv) {
            std::vector<Point> pts;

            for (const long long idx : curv)
                if (idx > 0 && static_cast<size_t>(idx) <= verts.size())
                    pts.push_back(verts[static_cast<size_t>(idx - 1)]);

            if (pts.size() >= 2)
                polylines.emplace_back(pts);

            in_curv = false;
        }
    }

    return polylines;
}

} // namespace file_obj
} // namespace session_cpp
