#include "trimmedsurface.h"
#include "trimesh_delaunay.h"
#include "fmt/core.h"
#include <fstream>
#include "trimmedsurface.pb.h"

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// Constructors
///////////////////////////////////////////////////////////////////////////////////////////

TrimmedSurface::TrimmedSurface() {}

TrimmedSurface::TrimmedSurface(const TrimmedSurface& other) {
    deep_copy_from(other);
}

TrimmedSurface& TrimmedSurface::operator=(const TrimmedSurface& other) {
    if (this != &other) deep_copy_from(other);
    return *this;
}

bool TrimmedSurface::operator==(const TrimmedSurface& other) const {
    if (name != other.name) return false;
    if (width != other.width) return false;
    if (surfacecolor != other.surfacecolor) return false;
    if (xform != other.xform) return false;
    if (m_surface != other.m_surface) return false;
    return true;
}

bool TrimmedSurface::operator!=(const TrimmedSurface& other) const {
    return !(*this == other);
}

TrimmedSurface::~TrimmedSurface() {}

void TrimmedSurface::deep_copy_from(const TrimmedSurface& src) {
    guid = ::guid();
    name = src.name;
    width = src.width;
    surfacecolor = src.surfacecolor;
    xform = src.xform;
    m_surface = src.m_surface;
    m_outer_loop = src.m_outer_loop;
    m_inner_loops = src.m_inner_loops;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Static Factory Methods
///////////////////////////////////////////////////////////////////////////////////////////

TrimmedSurface TrimmedSurface::create(const NurbsSurface& surface, const NurbsCurve& outer_loop) {
    TrimmedSurface ts;
    ts.m_surface = surface;
    ts.m_outer_loop = outer_loop;
    return ts;
}

TrimmedSurface TrimmedSurface::create_planar(const NurbsCurve& boundary) {
    NurbsSurface srf = NurbsSurface::create_planar(boundary);
    TrimmedSurface ts;
    ts.m_surface = srf;
    ts.m_outer_loop = srf.get_outer_loop();
    ts.m_inner_loops.clear();
    for (int i = 0; i < srf.inner_loop_count(); ++i)
        ts.m_inner_loops.push_back(srf.get_inner_loop(i));
    return ts;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////////////////

NurbsSurface TrimmedSurface::surface() const { return m_surface; }
NurbsCurve TrimmedSurface::get_outer_loop() const { return m_outer_loop; }
void TrimmedSurface::set_outer_loop(const NurbsCurve& loop) { m_outer_loop = loop; }
bool TrimmedSurface::is_trimmed() const { return m_outer_loop.is_valid(); }
bool TrimmedSurface::is_valid() const { return m_surface.is_valid(); }

///////////////////////////////////////////////////////////////////////////////////////////
// Inner Loops
///////////////////////////////////////////////////////////////////////////////////////////

void TrimmedSurface::add_inner_loop(const NurbsCurve& loop_2d) {
    m_inner_loops.push_back(loop_2d);
}

void TrimmedSurface::add_hole(const NurbsCurve& curve_3d) {
    NurbsSurface temp = m_surface;
    temp.add_hole(curve_3d);
    if (temp.inner_loop_count() > 0)
        m_inner_loops.push_back(temp.get_inner_loop(temp.inner_loop_count() - 1));
}

void TrimmedSurface::add_holes(const std::vector<NurbsCurve>& curves_3d) {
    for (const auto& crv : curves_3d) add_hole(crv);
}

NurbsCurve TrimmedSurface::get_inner_loop(int index) const { return m_inner_loops[index]; }
int TrimmedSurface::inner_loop_count() const { return static_cast<int>(m_inner_loops.size()); }
void TrimmedSurface::clear_inner_loops() { m_inner_loops.clear(); }

///////////////////////////////////////////////////////////////////////////////////////////
// Evaluation
///////////////////////////////////////////////////////////////////////////////////////////

Point TrimmedSurface::point_at(double u, double v) const { return m_surface.point_at(u, v); }
Vector TrimmedSurface::normal_at(double u, double v) const { return m_surface.normal_at(u, v); }

///////////////////////////////////////////////////////////////////////////////////////////
// Meshing
///////////////////////////////////////////////////////////////////////////////////////////

Mesh TrimmedSurface::mesh() const {
    NurbsSurface temp = m_surface;
    if (is_trimmed()) temp.set_outer_loop(m_outer_loop);
    temp.clear_inner_loops();
    for (const auto& loop : m_inner_loops) temp.add_inner_loop(loop);
    return temp.mesh();
}

///////////////////////////////////////////////////////////////////////////////////////////
// Transformation
///////////////////////////////////////////////////////////////////////////////////////////

void TrimmedSurface::transform() {
    m_surface.xform = xform;
    m_surface.transform();
    xform = Xform::identity();
}

TrimmedSurface TrimmedSurface::transformed() const {
    TrimmedSurface ts = *this;
    ts.transform();
    return ts;
}

///////////////////////////////////////////////////////////////////////////////////////////
// JSON Serialization
///////////////////////////////////////////////////////////////////////////////////////////

nlohmann::ordered_json TrimmedSurface::jsondump() const {
    nlohmann::ordered_json j;
    j["guid"] = guid;
    j["inner_loops"] = nlohmann::ordered_json::array();
    for (const auto& loop : m_inner_loops)
        j["inner_loops"].push_back(loop.jsondump());
    j["name"] = name;
    if (m_outer_loop.is_valid())
        j["outer_loop"] = m_outer_loop.jsondump();
    j["surface"] = m_surface.jsondump();
    j["surfacecolor"] = surfacecolor.jsondump();
    j["type"] = "TrimmedSurface";
    j["width"] = width;
    j["xform"] = xform.jsondump();
    return j;
}

TrimmedSurface TrimmedSurface::jsonload(const nlohmann::json& data) {
    TrimmedSurface ts;
    if (data.contains("guid")) ts.guid = data["guid"];
    if (data.contains("name")) ts.name = data["name"];
    if (data.contains("width")) ts.width = data["width"];
    if (data.contains("surfacecolor")) ts.surfacecolor = Color::jsonload(data["surfacecolor"]);
    if (data.contains("xform")) ts.xform = Xform::jsonload(data["xform"]);
    if (data.contains("surface")) ts.m_surface = NurbsSurface::jsonload(data["surface"]);
    if (data.contains("outer_loop")) ts.m_outer_loop = NurbsCurve::jsonload(data["outer_loop"]);
    if (data.contains("inner_loops")) {
        for (const auto& loop_data : data["inner_loops"])
            ts.m_inner_loops.push_back(NurbsCurve::jsonload(loop_data));
    }
    return ts;
}

std::string TrimmedSurface::json_dumps() const { return jsondump().dump(); }
TrimmedSurface TrimmedSurface::json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void TrimmedSurface::json_dump(const std::string& filename) const {
    std::ofstream file(filename);
    file << jsondump().dump(4);
}

TrimmedSurface TrimmedSurface::json_load(const std::string& filename) {
    std::ifstream file(filename);
    nlohmann::json data;
    file >> data;
    return jsonload(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Protobuf Serialization
///////////////////////////////////////////////////////////////////////////////////////////

std::string TrimmedSurface::pb_dumps() const {
    session_proto::TrimmedSurface proto;
    proto.set_guid(guid);
    proto.set_name(name);
    proto.set_width(width);

    // Surface
    std::string srf_data = m_surface.pb_dumps();
    auto* srf_proto = proto.mutable_surface();
    srf_proto->ParseFromString(srf_data);

    // Outer loop
    if (is_trimmed()) {
        std::string loop_data = m_outer_loop.pb_dumps();
        auto* ol = proto.mutable_outer_loop();
        ol->ParseFromString(loop_data);
    }

    // Inner loops
    for (const auto& inner : m_inner_loops) {
        std::string loop_data = inner.pb_dumps();
        auto* il = proto.add_inner_loops();
        il->ParseFromString(loop_data);
    }

    // Color
    auto* color_proto = proto.mutable_surfacecolor();
    color_proto->set_name(surfacecolor.name);
    color_proto->set_r(surfacecolor.r);
    color_proto->set_g(surfacecolor.g);
    color_proto->set_b(surfacecolor.b);
    color_proto->set_a(surfacecolor.a);

    // Transform
    auto* xform_proto = proto.mutable_xform();
    xform_proto->set_guid(xform.guid);
    xform_proto->set_name(xform.name);
    for (int i = 0; i < 16; ++i)
        xform_proto->add_matrix(xform.m[i]);

    return proto.SerializeAsString();
}

TrimmedSurface TrimmedSurface::pb_loads(const std::string& data) {
    session_proto::TrimmedSurface proto;
    proto.ParseFromString(data);

    TrimmedSurface ts;
    ts.guid = proto.guid();
    ts.name = proto.name();
    ts.width = proto.width();

    // Surface
    if (proto.has_surface()) {
        std::string srf_data = proto.surface().SerializeAsString();
        ts.m_surface = NurbsSurface::pb_loads(srf_data);
    }

    // Outer loop
    if (proto.has_outer_loop()) {
        std::string loop_data = proto.outer_loop().SerializeAsString();
        ts.m_outer_loop = NurbsCurve::pb_loads(loop_data);
    }

    // Inner loops
    for (int i = 0; i < proto.inner_loops_size(); ++i) {
        std::string loop_data = proto.inner_loops(i).SerializeAsString();
        ts.m_inner_loops.push_back(NurbsCurve::pb_loads(loop_data));
    }

    // Color
    const auto& color_proto = proto.surfacecolor();
    ts.surfacecolor.name = color_proto.name();
    ts.surfacecolor.r = color_proto.r();
    ts.surfacecolor.g = color_proto.g();
    ts.surfacecolor.b = color_proto.b();
    ts.surfacecolor.a = color_proto.a();

    // Transform
    const auto& xform_proto = proto.xform();
    ts.xform.guid = xform_proto.guid();
    ts.xform.name = xform_proto.name();
    for (int i = 0; i < 16 && i < xform_proto.matrix_size(); ++i)
        ts.xform.m[i] = xform_proto.matrix(i);

    return ts;
}

void TrimmedSurface::pb_dump(const std::string& filename) const {
    std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

TrimmedSurface TrimmedSurface::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)),
                     std::istreambuf_iterator<char>());
    return pb_loads(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// String Representation
///////////////////////////////////////////////////////////////////////////////////////////

std::string TrimmedSurface::str() const {
    return fmt::format("TrimmedSurface(name={}, trimmed={}, holes={})",
                       name, is_trimmed() ? "true" : "false", inner_loop_count());
}

std::string TrimmedSurface::repr() const {
    return fmt::format("TrimmedSurface(\n  name={},\n  trimmed={},\n  holes={},\n  surface={}\n)",
                       name, is_trimmed() ? "true" : "false", inner_loop_count(), m_surface.str());
}

std::ostream& operator<<(std::ostream& os, const TrimmedSurface& ts) {
    os << ts.str();
    return os;
}

} // namespace session_cpp
