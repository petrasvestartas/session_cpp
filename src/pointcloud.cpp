#include "pointcloud.h"
#include "guid.h"
#include "tolerance.h"
#include <fstream>
#include <sstream>

#ifdef ENABLE_PROTOBUF
#include "pointcloud.pb.h"
#include "xform.pb.h"
#endif

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// Constructors
///////////////////////////////////////////////////////////////////////////////////////////

PointCloud::PointCloud() {
    xform = Xform::identity();
}

PointCloud::PointCloud(const std::vector<Point>& points,
                       const std::vector<Vector>& normals,
                       const std::vector<Color>& colors) {
    xform = Xform::identity();

    _coords.reserve(points.size() * 3);
    for (const auto& p : points) {
        _coords.push_back(p[0]);
        _coords.push_back(p[1]);
        _coords.push_back(p[2]);
    }

    _colors.reserve(colors.size() * 4);
    for (const auto& c : colors) {
        _colors.push_back(c.r);
        _colors.push_back(c.g);
        _colors.push_back(c.b);
        _colors.push_back(c.a);
    }

    _normals.reserve(normals.size() * 3);
    for (const auto& n : normals) {
        _normals.push_back(n[0]);
        _normals.push_back(n[1]);
        _normals.push_back(n[2]);
    }
}

PointCloud::PointCloud(const PointCloud& other)
    : guid(::guid()),
      name(other.name),
      point_size(other.point_size),
      xform(other.xform),
      _coords(other._coords),
      _colors(other._colors),
      _normals(other._normals) {
}

PointCloud& PointCloud::operator=(const PointCloud& other) {
    if (this != &other) {
        guid = ::guid();
        name = other.name;
        point_size = other.point_size;
        xform = other.xform;
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

///////////////////////////////////////////////////////////////////////////////////////////
// Point Access
///////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////
// Color Access
///////////////////////////////////////////////////////////////////////////////////////////

Color PointCloud::get_color(size_t index) const {
    size_t idx = index * 4;
    return Color(_colors[idx], _colors[idx + 1], _colors[idx + 2], _colors[idx + 3]);
}

void PointCloud::set_color(size_t index, const Color& color) {
    size_t idx = index * 4;
    _colors[idx] = color.r;
    _colors[idx + 1] = color.g;
    _colors[idx + 2] = color.b;
    _colors[idx + 3] = color.a;
}

void PointCloud::add_color(const Color& color) {
    _colors.push_back(color.r);
    _colors.push_back(color.g);
    _colors.push_back(color.b);
    _colors.push_back(color.a);
}

std::vector<Color> PointCloud::get_colors() const {
    std::vector<Color> colors;
    colors.reserve(color_count());
    for (size_t i = 0; i < color_count(); ++i) {
        size_t idx = i * 4;
        colors.emplace_back(_colors[idx], _colors[idx + 1], _colors[idx + 2], _colors[idx + 3]);
    }
    return colors;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Normal Access
///////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////
// String Representations
///////////////////////////////////////////////////////////////////////////////////////////

std::string PointCloud::str() const {
    return fmt::format("{} points", point_count());
}

std::string PointCloud::repr() const {
    return fmt::format("PointCloud({}, {} points, {} colors, {} normals)",
                       name, point_count(), color_count(), normal_count());
}

///////////////////////////////////////////////////////////////////////////////////////////
// Equality
///////////////////////////////////////////////////////////////////////////////////////////

bool PointCloud::operator==(const PointCloud& other) const {
    return name == other.name &&
           _coords == other._coords &&
           _colors == other._colors &&
           _normals == other._normals;
}

bool PointCloud::operator!=(const PointCloud& other) const {
    return !(*this == other);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Transformation
///////////////////////////////////////////////////////////////////////////////////////////

void PointCloud::transform() {
    for (size_t i = 0; i < point_count(); ++i) {
        size_t idx = i * 3;
        Point pt(_coords[idx], _coords[idx + 1], _coords[idx + 2]);
        xform.transform_point(pt);
        _coords[idx] = pt[0];
        _coords[idx + 1] = pt[1];
        _coords[idx + 2] = pt[2];
    }

    for (size_t i = 0; i < normal_count(); ++i) {
        size_t idx = i * 3;
        Vector n(_normals[idx], _normals[idx + 1], _normals[idx + 2]);
        xform.transform_vector(n);
        _normals[idx] = n[0];
        _normals[idx + 1] = n[1];
        _normals[idx + 2] = n[2];
    }

    xform = Xform::identity();
}

PointCloud PointCloud::transformed() const {
    PointCloud result = *this;
    result.transform();
    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////
// No-copy Operators
///////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////
// Copy Operators
///////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////
// JSON Serialization
///////////////////////////////////////////////////////////////////////////////////////////

nlohmann::ordered_json PointCloud::jsondump() const {
    // Alphabetical order to match Rust's serde_json
    nlohmann::ordered_json data;
    data["colors"] = _colors;
    data["coords"] = _coords;
    data["guid"] = guid;
    data["name"] = name;
    data["normals"] = _normals;
    data["point_size"] = point_size;
    data["type"] = "PointCloud";
    data["xform"] = xform.jsondump();
    return data;
}

PointCloud PointCloud::jsonload(const nlohmann::json& data) {
    std::vector<double> coords = data.value("coords", std::vector<double>{});
    std::vector<int> colors = data.value("colors", std::vector<int>{});
    std::vector<double> normals = data.value("normals", std::vector<double>{});

    PointCloud pc = from_coords(coords, colors, normals);
    pc.guid = data.value("guid", pc.guid);
    pc.name = data.value("name", pc.name);
    pc.point_size = data.value("point_size", 1.0);

    if (data.contains("xform")) {
        pc.xform = Xform::jsonload(data["xform"]);
    }

    return pc;
}

void PointCloud::json_dump(const std::string& filename) const {
    std::ofstream ofs(filename);
    ofs << jsondump().dump(2);
    ofs.close();
}

PointCloud PointCloud::json_load(const std::string& filename) {
    std::ifstream ifs(filename);
    nlohmann::json data;
    ifs >> data;
    ifs.close();
    return jsonload(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Protobuf Serialization
///////////////////////////////////////////////////////////////////////////////////////////

#ifdef ENABLE_PROTOBUF
std::string PointCloud::to_protobuf() const {
    session_proto::PointCloud proto;
    proto.set_guid(guid);
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

    auto* proto_xform = proto.mutable_xform();
    proto_xform->set_name(xform.name);
    for (int i = 0; i < 16; ++i) {
        proto_xform->add_matrix(xform.m[i]);
    }

    return proto.SerializeAsString();
}

PointCloud PointCloud::from_protobuf(const std::string& data) {
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
    pc.guid = proto.guid();
    pc.name = proto.name();
    pc.point_size = proto.point_size() > 0 ? proto.point_size() : 1.0;

    if (proto.has_xform()) {
        pc.xform.name = proto.xform().name();
        for (int i = 0; i < proto.xform().matrix_size() && i < 16; ++i) {
            pc.xform.m[i] = proto.xform().matrix(i);
        }
    }

    return pc;
}

void PointCloud::protobuf_dump(const std::string& filename) const {
    std::ofstream ofs(filename, std::ios::binary);
    ofs << to_protobuf();
    ofs.close();
}

PointCloud PointCloud::protobuf_load(const std::string& filename) {
    std::ifstream ifs(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(ifs)),
                      std::istreambuf_iterator<char>());
    ifs.close();
    return from_protobuf(data);
}
#else
std::string PointCloud::to_protobuf() const {
    throw std::runtime_error("Protobuf support not enabled");
}

PointCloud PointCloud::from_protobuf(const std::string& data) {
    (void)data;
    throw std::runtime_error("Protobuf support not enabled");
}

void PointCloud::protobuf_dump(const std::string& filename) const {
    (void)filename;
    throw std::runtime_error("Protobuf support not enabled");
}

PointCloud PointCloud::protobuf_load(const std::string& filename) {
    (void)filename;
    throw std::runtime_error("Protobuf support not enabled");
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////
// Stream operator
///////////////////////////////////////////////////////////////////////////////////////////

std::ostream& operator<<(std::ostream& os, const PointCloud& cloud) {
    return os << cloud.repr();
}

} // namespace session_cpp
