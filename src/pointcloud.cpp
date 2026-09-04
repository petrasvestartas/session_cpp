#include "pointcloud.h"
#include "guid.h"
#include "spatial_octree.h"
#include "tolerance.h"
#include <cmath>
#include <fstream>
#include <sstream>

#include "pointcloud.pb.h"

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════

PointCloud::PointCloud() {
}

PointCloud::PointCloud(const std::vector<Point>& points,
                       const std::vector<Vector>& normals,
                       const std::vector<Color>& colors) {
    _coords.reserve(points.size() * 3);
    for (const auto& p : points) {
        _coords.push_back(p[0]);
        _coords.push_back(p[1]);
        _coords.push_back(p[2]);
    }

    _colors.reserve(colors.size() * 4);
    for (const auto& c : colors) {
        _colors.push_back(static_cast<int>(std::lround(c.r * 255.0)));
        _colors.push_back(static_cast<int>(std::lround(c.g * 255.0)));
        _colors.push_back(static_cast<int>(std::lround(c.b * 255.0)));
        _colors.push_back(static_cast<int>(std::lround(c.a * 255.0)));
    }

    _normals.reserve(normals.size() * 3);
    for (const auto& n : normals) {
        _normals.push_back(n[0]);
        _normals.push_back(n[1]);
        _normals.push_back(n[2]);
    }
}

PointCloud::PointCloud(const PointCloud& other)
    :
      name(other.name),
      point_size(other.point_size),
      _coords(other._coords),
      _colors(other._colors),
      _normals(other._normals) {
}

PointCloud& PointCloud::operator=(const PointCloud& other) {
    if (this != &other) {
        _guid.clear();
        name = other.name;
        point_size = other.point_size;
        _coords = other._coords;
        _colors = other._colors;
        _normals = other._normals;
    }
    return *this;
}

PointCloud PointCloud::from_coords(const std::vector<double>& coords,
                                   const std::vector<int>& colors,
                                   const std::vector<double>& normals) {
    PointCloud pc;
    pc._coords = coords;
    pc._colors = colors;
    pc._normals = normals;
    return pc;
}

// ═══════════════════════════════════════════════════════════════════════════
// Point Access
// ═══════════════════════════════════════════════════════════════════════════

Point PointCloud::get_point(size_t index) const {
    size_t idx = index * 3;
    return Point(_coords[idx], _coords[idx + 1], _coords[idx + 2]);
}

void PointCloud::set_point(size_t index, const Point& point) {
    size_t idx = index * 3;
    _coords[idx] = point[0];
    _coords[idx + 1] = point[1];
    _coords[idx + 2] = point[2];
}

void PointCloud::add_point(const Point& point) {
    _coords.push_back(point[0]);
    _coords.push_back(point[1]);
    _coords.push_back(point[2]);
}

std::vector<Point> PointCloud::get_points() const {
    std::vector<Point> points;
    points.reserve(point_count());
    for (size_t i = 0; i < point_count(); ++i) {
        size_t idx = i * 3;
        points.emplace_back(_coords[idx], _coords[idx + 1], _coords[idx + 2]);
    }
    return points;
}

// ═══════════════════════════════════════════════════════════════════════════
// Color Access
// ═══════════════════════════════════════════════════════════════════════════

Color PointCloud::get_color(size_t index) const {
    size_t idx = index * 4;
    // Stored 0-255 (the flat array is int, and the proto field is uint32); Color is 0-1.
    return Color(_colors[idx] / 255.0, _colors[idx + 1] / 255.0,
                 _colors[idx + 2] / 255.0, _colors[idx + 3] / 255.0);
}

void PointCloud::set_color(size_t index, const Color& color) {
    size_t idx = index * 4;
    _colors[idx] = static_cast<int>(std::lround(color.r * 255.0));
    _colors[idx + 1] = static_cast<int>(std::lround(color.g * 255.0));
    _colors[idx + 2] = static_cast<int>(std::lround(color.b * 255.0));
    _colors[idx + 3] = static_cast<int>(std::lround(color.a * 255.0));
}

void PointCloud::add_color(const Color& color) {
    _colors.push_back(static_cast<int>(std::lround(color.r * 255.0)));
    _colors.push_back(static_cast<int>(std::lround(color.g * 255.0)));
    _colors.push_back(static_cast<int>(std::lround(color.b * 255.0)));
    _colors.push_back(static_cast<int>(std::lround(color.a * 255.0)));
}

std::vector<Color> PointCloud::get_colors() const {
    std::vector<Color> colors;
    colors.reserve(color_count());
    for (size_t i = 0; i < color_count(); ++i) {
        size_t idx = i * 4;
        colors.emplace_back(_colors[idx] / 255.0, _colors[idx + 1] / 255.0,
                            _colors[idx + 2] / 255.0, _colors[idx + 3] / 255.0);
    }
    return colors;
}

// ═══════════════════════════════════════════════════════════════════════════
// Normal Access
// ═══════════════════════════════════════════════════════════════════════════

Vector PointCloud::get_normal(size_t index) const {
    size_t idx = index * 3;
    return Vector(_normals[idx], _normals[idx + 1], _normals[idx + 2]);
}

void PointCloud::set_normal(size_t index, const Vector& normal) {
    size_t idx = index * 3;
    _normals[idx] = normal[0];
    _normals[idx + 1] = normal[1];
    _normals[idx + 2] = normal[2];
}

void PointCloud::add_normal(const Vector& normal) {
    _normals.push_back(normal[0]);
    _normals.push_back(normal[1]);
    _normals.push_back(normal[2]);
}

std::vector<Vector> PointCloud::get_normals() const {
    std::vector<Vector> normals;
    normals.reserve(normal_count());
    for (size_t i = 0; i < normal_count(); ++i) {
        size_t idx = i * 3;
        normals.emplace_back(_normals[idx], _normals[idx + 1], _normals[idx + 2]);
    }
    return normals;
}

// ═══════════════════════════════════════════════════════════════════════════
// LOD Octree
// ═══════════════════════════════════════════════════════════════════════════

void PointCloud::build_lod(double root_spacing, int leaf_capacity) {
    SpatialOctree tree = SpatialOctree::from_coords(_coords, root_spacing, leaf_capacity);
    const std::vector<int>& order = tree.order();

    // Identity is minted HERE, before the first permutation, so an id records where a point
    // began. After this the ids travel with the points and the index is free to move.
    if (_point_ids.empty()) {
        _point_ids.resize(_coords.size() / 3);
        for (size_t k = 0; k < _point_ids.size(); ++k) {
            _point_ids[k] = static_cast<int>(k);
        }
    }

    // Permute the three parallel arrays into octree order. This is what lets a node be one
    // (first, count) range, so `order` itself never has to be stored - 4 bytes a point.
    std::vector<double> coords;
    coords.reserve(_coords.size());
    std::vector<int> colors;
    colors.reserve(_colors.size());
    std::vector<double> normals;
    normals.reserve(_normals.size());
    std::vector<int> ids;
    ids.reserve(_point_ids.size());
    bool has_colors = _colors.size() == order.size() * 4;
    bool has_normals = _normals.size() == order.size() * 3;
    for (int idx : order) {
        ids.push_back(_point_ids[idx]);
        for (int k = 0; k < 3; ++k) {
            coords.push_back(_coords[idx * 3 + k]);
        }
        if (has_colors) {
            for (int k = 0; k < 4; ++k) {
                colors.push_back(_colors[idx * 4 + k]);
            }
        }
        if (has_normals) {
            for (int k = 0; k < 3; ++k) {
                normals.push_back(_normals[idx * 3 + k]);
            }
        }
    }
    _coords = std::move(coords);
    _point_ids = std::move(ids);
    if (has_colors) {
        _colors = std::move(colors);
    }
    if (has_normals) {
        _normals = std::move(normals);
    }

    int n = tree.node_count();
    _lod_min.clear();
    _lod_size.clear();
    _lod_spacing.clear();
    _lod_level.clear();
    _lod_first.clear();
    _lod_count.clear();
    _lod_children.clear();
    for (int i = 0; i < n; ++i) {
        std::pair<Point, double> cube = tree.node_cube(i);
        _lod_min.push_back(cube.first[0] - cube.second * 0.5);
        _lod_min.push_back(cube.first[1] - cube.second * 0.5);
        _lod_min.push_back(cube.first[2] - cube.second * 0.5);
        _lod_size.push_back(cube.second);
        _lod_spacing.push_back(tree.node_spacing(i));
        _lod_level.push_back(tree.node_level(i));
        std::pair<int, int> range = tree.node_range(i);
        _lod_first.push_back(range.first);
        _lod_count.push_back(range.second);
        std::vector<int> kids = tree.children(i);
        for (int k = 0; k < 8; ++k) {
            _lod_children.push_back(k < static_cast<int>(kids.size()) ? kids[k] : -1);
        }
    }
}

std::pair<Point, double> PointCloud::lod_cube(int i) const {
    double half = _lod_size[i] * 0.5;
    return {Point(_lod_min[i * 3] + half, _lod_min[i * 3 + 1] + half, _lod_min[i * 3 + 2] + half),
            _lod_size[i]};
}

int PointCloud::index_of_id(int id) const {
    if (_point_ids.empty()) {
        return id >= 0 && id < static_cast<int>(_coords.size() / 3) ? id : -1;
    }
    for (size_t k = 0; k < _point_ids.size(); ++k) {
        if (_point_ids[k] == id) {
            return static_cast<int>(k);
        }
    }
    return -1;
}

std::vector<int> PointCloud::lod_children(int i) const {
    return std::vector<int>(_lod_children.begin() + i * 8, _lod_children.begin() + i * 8 + 8);
}

// ═══════════════════════════════════════════════════════════════════════════
// String Representations
// ═══════════════════════════════════════════════════════════════════════════

std::string PointCloud::str() const {
    return fmt::format("{} points", point_count());
}

std::string PointCloud::repr() const {
    return fmt::format("PointCloud({}, {} points, {} colors, {} normals)",
                       name, point_count(), color_count(), normal_count());
}

// ═══════════════════════════════════════════════════════════════════════════
// Equality
// ═══════════════════════════════════════════════════════════════════════════

bool PointCloud::operator==(const PointCloud& other) const {
    return name == other.name &&
           _coords == other._coords &&
           _colors == other._colors &&
           _normals == other._normals &&
           _lod_first == other._lod_first &&
           _lod_count == other._lod_count &&
           _point_ids == other._point_ids;
}

bool PointCloud::operator!=(const PointCloud& other) const {
    return !(*this == other);
}

// ═══════════════════════════════════════════════════════════════════════════
// Transformation
// ═══════════════════════════════════════════════════════════════════════════

void PointCloud::transform(const Xform& xform) {
    for (size_t i = 0; i < point_count(); ++i) {
        size_t idx = i * 3;
        Point pt(_coords[idx], _coords[idx + 1], _coords[idx + 2]);
        pt.transform(xform);
        _coords[idx] = pt[0];
        _coords[idx + 1] = pt[1];
        _coords[idx + 2] = pt[2];
    }

    for (size_t i = 0; i < normal_count(); ++i) {
        size_t idx = i * 3;
        Vector n(_normals[idx], _normals[idx + 1], _normals[idx + 2]);
        n.transform(xform);
        _normals[idx] = n[0];
        _normals[idx + 1] = n[1];
        _normals[idx + 2] = n[2];
    }
}

PointCloud PointCloud::transformed(const Xform& xform) const {
    PointCloud result = *this;
    result.transform(xform);
    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// No-copy Operators
// ═══════════════════════════════════════════════════════════════════════════

PointCloud& PointCloud::operator+=(const Vector& v) {
    for (size_t i = 0; i < point_count(); ++i) {
        size_t idx = i * 3;
        _coords[idx] += v[0];
        _coords[idx + 1] += v[1];
        _coords[idx + 2] += v[2];
    }
    return *this;
}

PointCloud& PointCloud::operator-=(const Vector& v) {
    for (size_t i = 0; i < point_count(); ++i) {
        size_t idx = i * 3;
        _coords[idx] -= v[0];
        _coords[idx + 1] -= v[1];
        _coords[idx + 2] -= v[2];
    }
    return *this;
}

// ═══════════════════════════════════════════════════════════════════════════
// Copy Operators
// ═══════════════════════════════════════════════════════════════════════════

PointCloud PointCloud::operator+(const Vector& v) const {
    PointCloud result = *this;
    result += v;
    return result;
}

PointCloud PointCloud::operator-(const Vector& v) const {
    PointCloud result = *this;
    result -= v;
    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON Serialization
// ═══════════════════════════════════════════════════════════════════════════

nlohmann::ordered_json PointCloud::jsondump() const {
    // Alphabetical order to match Rust's serde_json
    nlohmann::ordered_json data;
    data["colors"] = _colors;
    data["coords"] = _coords;
    data["guid"] = guid();
    data["lod_children"] = _lod_children;
    data["lod_count"] = _lod_count;
    data["lod_first"] = _lod_first;
    data["lod_level"] = _lod_level;
    data["lod_min"] = _lod_min;
    data["lod_size"] = _lod_size;
    data["lod_spacing"] = _lod_spacing;
    data["name"] = name;
    data["normals"] = _normals;
    data["point_ids"] = _point_ids;
    data["point_size"] = point_size;
    data["type"] = "PointCloud";
    return data;
}

PointCloud PointCloud::jsonload(const nlohmann::json& data) {
    std::vector<double> coords = data.value("coords", std::vector<double>{});
    std::vector<int> colors = data.value("colors", std::vector<int>{});
    std::vector<double> normals = data.value("normals", std::vector<double>{});

    PointCloud pc = from_coords(coords, colors, normals);
    pc.guid() = data.value("guid", pc.guid());
    pc.name = data.value("name", pc.name);
    pc.point_size = data.value("point_size", 1.0);
    pc._lod_min = data.value("lod_min", std::vector<double>{});
    pc._lod_size = data.value("lod_size", std::vector<double>{});
    pc._lod_spacing = data.value("lod_spacing", std::vector<double>{});
    pc._lod_level = data.value("lod_level", std::vector<int>{});
    pc._lod_first = data.value("lod_first", std::vector<int>{});
    pc._lod_count = data.value("lod_count", std::vector<int>{});
    pc._lod_children = data.value("lod_children", std::vector<int>{});
    pc._point_ids = data.value("point_ids", std::vector<int>{});

    return pc;
}

std::string PointCloud::file_json_dumps() const {
    return jsondump().dump();
}

PointCloud PointCloud::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void PointCloud::file_json_dump(const std::string& filename) const {
    std::ofstream ofs(filename);
    ofs << jsondump().dump(2);
    ofs.close();
}

PointCloud PointCloud::file_json_load(const std::string& filename) {
    std::ifstream ifs(filename);
    nlohmann::json data;
    ifs >> data;
    ifs.close();
    return jsonload(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf Serialization
// ═══════════════════════════════════════════════════════════════════════════

std::string PointCloud::pb_dumps() const {
    session_proto::PointCloud proto;
    proto.set_guid(guid());
    proto.set_name(name);
    proto.set_point_size(point_size);

    for (double c : _coords) {
        proto.add_coords(c);
    }
    for (int c : _colors) {
        proto.add_colors(static_cast<uint32_t>(c));
    }
    for (double n : _normals) {
        proto.add_normals(n);
    }
    for (double v : _lod_min) {
        proto.add_lod_min(v);
    }
    for (double v : _lod_size) {
        proto.add_lod_size(v);
    }
    for (double v : _lod_spacing) {
        proto.add_lod_spacing(v);
    }
    for (int v : _lod_level) {
        proto.add_lod_level(v);
    }
    for (int v : _lod_first) {
        proto.add_lod_first(v);
    }
    for (int v : _lod_count) {
        proto.add_lod_count(v);
    }
    for (int v : _lod_children) {
        proto.add_lod_children(v);
    }
    for (int v : _point_ids) {
        proto.add_point_ids(static_cast<uint32_t>(v));
    }

    return proto.SerializeAsString();
}

PointCloud PointCloud::pb_loads(const std::string& data) {
    session_proto::PointCloud proto;
    proto.ParseFromString(data);

    std::vector<double> coords(proto.coords().begin(), proto.coords().end());
    std::vector<int> colors;
    colors.reserve(proto.colors_size());
    for (int i = 0; i < proto.colors_size(); ++i) {
        colors.push_back(static_cast<int>(proto.colors(i)));
    }
    std::vector<double> normals(proto.normals().begin(), proto.normals().end());

    PointCloud pc = from_coords(coords, colors, normals);
    pc.guid() = proto.guid();
    pc.name = proto.name();
    pc.point_size = proto.point_size() > 0 ? proto.point_size() : 1.0;
    pc._lod_min.assign(proto.lod_min().begin(), proto.lod_min().end());
    pc._lod_size.assign(proto.lod_size().begin(), proto.lod_size().end());
    pc._lod_spacing.assign(proto.lod_spacing().begin(), proto.lod_spacing().end());
    pc._lod_level.assign(proto.lod_level().begin(), proto.lod_level().end());
    pc._lod_first.assign(proto.lod_first().begin(), proto.lod_first().end());
    pc._lod_count.assign(proto.lod_count().begin(), proto.lod_count().end());
    pc._lod_children.assign(proto.lod_children().begin(), proto.lod_children().end());
    pc._point_ids.assign(proto.point_ids().begin(), proto.point_ids().end());

    return pc;
}

void PointCloud::pb_dump(const std::string& filename) const {
    std::ofstream ofs(filename, std::ios::binary);
    ofs << pb_dumps();
    ofs.close();
}

PointCloud PointCloud::pb_load(const std::string& filename) {
    std::ifstream ifs(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(ifs)),
                      std::istreambuf_iterator<char>());
    ifs.close();
    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// Stream operator
// ═══════════════════════════════════════════════════════════════════════════

std::ostream& operator<<(std::ostream& os, const PointCloud& cloud) {
    return os << cloud.repr();
}

} // namespace session_cpp
