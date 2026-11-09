#include "text.h"
#include "objects.h"
#include "objects.pb.h"
#include <fstream>
#include <stdexcept>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
Text::Text(const std::string& text, const Point& at) : text(text), at(at) {}

Text Text::from_tex(const std::string& text, const Point& at, bool spatial) {

    Text result(text, at);
    result.math = true;
    result.spatial = spatial;
    result.height = spatial ? 18.0 : 28.0;
    result.outline = spatial ? -1.5 : 0.0;
    result.serif = !spatial;

    return result;
}

Text::Text(const Text& other)
    : name(other.name), text(other.text), at(other.at), height(other.height), color(other.color),
      outline(other.outline), math(other.math), spatial(other.spatial), serif(other.serif) {}

Text& Text::operator=(const Text& other) {

    if (this == &other)
        return *this;

    _guid.clear();
    name = other.name;
    text = other.text;
    at = other.at;
    height = other.height;
    color = other.color;
    outline = other.outline;
    math = other.math;
    spatial = other.spatial;
    serif = other.serif;

    return *this;
}

Text Text::duplicate() const {
    return Text(*this);
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
bool Text::operator==(const Text& other) const {
    return name == other.name && text == other.text && at == other.at && height == other.height &&
           color == other.color && outline == other.outline && math == other.math &&
           spatial == other.spatial && serif == other.serif;
}

bool Text::operator!=(const Text& other) const {
    return !(*this == other);
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════
nlohmann::ordered_json Text::jsondump() const {

    nlohmann::ordered_json data;
    data["at"] = at.jsondump();
    data["color"] = color.jsondump();
    data["guid"] = guid();
    data["height"] = height;
    data["math"] = math;
    data["name"] = name;
    data["outline"] = outline;
    data["serif"] = serif;
    data["spatial"] = spatial;
    data["text"] = text;
    data["type"] = "Text";

    return data;
}

Text Text::jsonload(const nlohmann::json& data) {

    Text result(data.value("text", ""));
    result.guid() = data.value("guid", ::guid());
    result.name = data.value("name", "my_text");
    result.height = data.value("height", 28.0);
    result.outline = data.value("outline", 0.0);
    result.math = data.value("math", false);
    result.spatial = data.value("spatial", false);
    result.serif = data.value("serif", false);

    if (data.contains("at"))
        result.at = Point::jsonload(data["at"]);

    if (data.contains("color"))
        result.color = Color::jsonload(data["color"]);

    return result;
}

std::string Text::file_json_dumps() const {
    return jsondump().dump();
}

Text Text::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Text::file_json_dump(const std::string& filename) const {

    std::ofstream file(filename);

    if (!file)
        throw std::runtime_error("Failed to open JSON file: " + filename);

    file << jsondump().dump(4);

    if (!file)
        throw std::runtime_error("Failed to write JSON file: " + filename);
}

Text Text::file_json_load(const std::string& filename) {

    std::ifstream file(filename);

    if (!file)
        throw std::runtime_error("Failed to open JSON file: " + filename);

    return jsonload(nlohmann::ordered_json::parse(file));
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════
session_proto::Component Text::to_proto() const {
    return to_component().to_proto();
}

Text Text::from_proto(const session_proto::Component& proto) {
    return from_component(Component::from_proto(proto));
}

std::string Text::pb_dumps() const {
    return to_proto().SerializeAsString();
}

Text Text::pb_loads(const std::string& data) {

    session_proto::Component proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse Text protobuf data");

    return from_proto(proto);
}

void Text::pb_dump(const std::string& filename) const {

    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);

    if (!file)
        throw std::runtime_error("Failed to open protobuf file: " + filename);

    file.write(data.data(), data.size());

    if (!file)
        throw std::runtime_error("Failed to write protobuf file: " + filename);
}

Text Text::pb_load(const std::string& filename) {

    std::ifstream file(filename, std::ios::binary);

    if (!file)
        throw std::runtime_error("Failed to open protobuf file: " + filename);

    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    if (file.bad())
        throw std::runtime_error("Failed to read protobuf file: " + filename);

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// Component
// ═══════════════════════════════════════════════════════════════════════════
Component Text::to_component() const {

    Component component;
    component.type_name = "Text";
    component.guid() = guid();
    component.name = name;
    component.extra = jsondump();
    component.extra.erase("guid");
    component.extra.erase("name");
    component.extra.erase("type");

    return component;
}

Text Text::from_component(const Component& component) {

    nlohmann::ordered_json data = component.extra;
    data["guid"] = component.guid();
    data["name"] = component.name;

    return jsonload(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
std::string Text::str() const {
    return text;
}

std::string Text::repr() const {
    return "Text(" + jsondump().dump() + ")";
}

} // namespace session_cpp
