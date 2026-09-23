#include "objects.h"
#include "objects.pb.h"

namespace session_cpp {

namespace {

/// One list, duplicated: new vector, new objects, same guids.
template <class T>
std::shared_ptr<std::vector<std::shared_ptr<T>>> clone_list(
    const std::shared_ptr<std::vector<std::shared_ptr<T>>>& source
) {

    std::shared_ptr<std::vector<std::shared_ptr<T>>> out = std::make_shared<std::vector<std::shared_ptr<T>>>();

    if (!source)
        return out;

    out->reserve(source->size());

    for (const std::shared_ptr<T>& item : *source) {
        if (!item) {
            out->push_back(nullptr);
            continue;
        }

        std::shared_ptr<T> copy = std::make_shared<T>(*item);

        if (item->has_guid())
            copy->guid() = item->guid();

        out->push_back(std::move(copy));
    }

    return out;
}

/// Elements are polymorphic, so the copy goes through the virtual clone.
std::shared_ptr<std::vector<std::shared_ptr<Element>>> clone_elements(
    const std::shared_ptr<std::vector<std::shared_ptr<Element>>>& source
) {

    std::shared_ptr<std::vector<std::shared_ptr<Element>>> out =
        std::make_shared<std::vector<std::shared_ptr<Element>>>();

    if (!source)
        return out;

    out->reserve(source->size());

    for (const std::shared_ptr<Element>& item : *source) {
        if (!item) {
            out->push_back(nullptr);
            continue;
        }

        std::shared_ptr<Element> copy = item->clone();

        if (item->has_guid())
            copy->guid() = item->guid();

        out->push_back(std::move(copy));
    }

    return out;
}

/// A load is not a duplicate: the loaded object is moved in, feature guids included, and its guid put back.
template <class T> std::shared_ptr<T> keep_guid(T&& loaded) {
    const std::string guid = loaded.guid();
    std::shared_ptr<T> out = std::make_shared<T>(std::move(loaded));
    out->guid() = guid;

    return out;
}

/// Serialize every object of a list to JSON.
template <class T> std::vector<nlohmann::ordered_json> dump_list(const std::vector<std::shared_ptr<T>>& list) {

    std::vector<nlohmann::ordered_json> out;
    out.reserve(list.size());

    for (const std::shared_ptr<T>& item : list)
        out.push_back(item->jsondump());

    return out;
}

/// Load every object under key into the list, keeping guids.
template <class T> void load_list(const nlohmann::json& data, const char* key, std::vector<std::shared_ptr<T>>& list) {

    if (!data.contains(key))
        return;

    list.reserve(data[key].size());

    for (const nlohmann::json& item : data[key])
        list.push_back(keep_guid(T::jsonload(item)));
}

/// Serialize every object of a list into a repeated proto field.
template <class T, class R> void dump_pb_list(const std::vector<std::shared_ptr<T>>& list, R* repeated) {
    for (const std::shared_ptr<T>& item : list)
        repeated->Add()->ParseFromString(item->pb_dumps());
}

/// Load every message of a repeated proto field into the list, keeping guids.
template <class T, class R> void load_pb_list(const R& repeated, std::vector<std::shared_ptr<T>>& list) {
    for (const typename R::value_type& item : repeated)
        list.push_back(keep_guid(T::pb_loads(item.SerializeAsString())));
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Component
// ═══════════════════════════════════════════════════════════════════════════

nlohmann::ordered_json Component::jsondump() const {

    nlohmann::ordered_json data = extra;
    data["type"] = type_name;
    data["guid"] = guid();
    data["name"] = name;

    return data;
}

Component Component::jsonload(const nlohmann::json& data) {

    Component component;
    component.type_name = data.value("type", "");
    component.guid() = data.value("guid", ::guid());
    component.name = data.value("name", "my_component");
    component.extra = data;
    component.extra.erase("type");
    component.extra.erase("guid");
    component.extra.erase("name");

    return component;
}

std::string Component::pb_dumps() const {

    session_proto::Component proto;
    proto.set_type_name(type_name);
    proto.set_guid(guid());
    proto.set_name(name);
    proto.set_json_data(extra.dump());

    return proto.SerializeAsString();
}

Component Component::pb_loads(const std::string& data) {

    session_proto::Component proto;
    proto.ParseFromString(data);
    Component component;
    component.type_name = proto.type_name();
    component.guid() = proto.guid();
    component.name = proto.name();
    component.extra = nlohmann::ordered_json::parse(proto.json_data(), nullptr, false);

    return component;
}

// ═══════════════════════════════════════════════════════════════════════════
// Objects
// ═══════════════════════════════════════════════════════════════════════════

Objects::Objects(std::string name) : name(std::move(name)) {

    points = std::make_shared<std::vector<std::shared_ptr<Point>>>();
    lines = std::make_shared<std::vector<std::shared_ptr<Line>>>();
    planes = std::make_shared<std::vector<std::shared_ptr<Plane>>>();
    bboxes = std::make_shared<std::vector<std::shared_ptr<OBB>>>();
    polylines = std::make_shared<std::vector<std::shared_ptr<Polyline>>>();
    pointclouds = std::make_shared<std::vector<std::shared_ptr<PointCloud>>>();
    meshes = std::make_shared<std::vector<std::shared_ptr<Mesh>>>();
    nurbscurves = std::make_shared<std::vector<std::shared_ptr<NurbsCurve>>>();
    nurbssurfaces = std::make_shared<std::vector<std::shared_ptr<NurbsSurface>>>();
    breps = std::make_shared<std::vector<std::shared_ptr<BRep>>>();
    elements = std::make_shared<std::vector<std::shared_ptr<Element>>>();
    components = std::make_shared<std::vector<Component>>();
    instances = std::make_shared<std::vector<std::shared_ptr<InstanceRef>>>();
}

Objects::Objects(const Objects& other) : name(other.name) {

    if (other.has_guid())
        guid() = other.guid();

    points = clone_list(other.points);
    lines = clone_list(other.lines);
    planes = clone_list(other.planes);
    bboxes = clone_list(other.bboxes);
    polylines = clone_list(other.polylines);
    pointclouds = clone_list(other.pointclouds);
    meshes = clone_list(other.meshes);
    nurbscurves = clone_list(other.nurbscurves);
    nurbssurfaces = clone_list(other.nurbssurfaces);
    breps = clone_list(other.breps);
    elements = clone_elements(other.elements);
    components =
        std::make_shared<std::vector<Component>>(other.components ? *other.components : std::vector<Component>{});
    instances = clone_list(other.instances);
}

Objects& Objects::operator=(const Objects& other) {

    if (this != &other) {
        Objects copy(other);
        *this = std::move(copy);
    }

    return *this;
}

std::string Objects::str() const {
    return fmt::format("Objects(name={}, guid={}, points={})", name, guid(), points->size());
}

nlohmann::ordered_json Objects::jsondump() const {

    std::vector<nlohmann::ordered_json> components_json;
    components_json.reserve(components->size());

    for (const Component& component : *components)
        components_json.push_back(component.jsondump());

    return nlohmann::ordered_json{
        {"type", "Objects"},
        {"guid", guid()},
        {"name", name},
        {"bboxes", dump_list(*bboxes)},
        {"breps", dump_list(*breps)},
        {"components", components_json},
        {"elements", dump_list(*elements)},
        {"instances", dump_list(*instances)},
        {"lines", dump_list(*lines)},
        {"meshes", dump_list(*meshes)},
        {"nurbscurves", dump_list(*nurbscurves)},
        {"nurbssurfaces", dump_list(*nurbssurfaces)},
        {"planes", dump_list(*planes)},
        {"pointclouds", dump_list(*pointclouds)},
        {"points", dump_list(*points)},
        {"polylines", dump_list(*polylines)}
    };
}

Objects Objects::jsonload(const nlohmann::json& data) {

    Objects objects(data.value("name", "my_objects"));
    objects.guid() = data.value("guid", ::guid());
    load_list(data, "bboxes", *objects.bboxes);
    load_list(data, "breps", *objects.breps);
    load_list(data, "elements", *objects.elements);
    load_list(data, "instances", *objects.instances);
    load_list(data, "lines", *objects.lines);
    load_list(data, "meshes", *objects.meshes);
    load_list(data, "nurbscurves", *objects.nurbscurves);
    load_list(data, "nurbssurfaces", *objects.nurbssurfaces);
    load_list(data, "planes", *objects.planes);
    load_list(data, "pointclouds", *objects.pointclouds);
    load_list(data, "points", *objects.points);
    load_list(data, "polylines", *objects.polylines);

    if (data.contains("components"))
        for (const nlohmann::json& item : data["components"])
            objects.components->push_back(Component::jsonload(item));

    return objects;
}

std::string Objects::file_json_dumps() const {
    return jsondump().dump();
}

Objects Objects::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Objects::file_json_dump(const std::string& filename) const {
    std::ofstream file(filename);
    file << jsondump().dump(4);
}

Objects Objects::file_json_load(const std::string& filename) {
    std::ifstream file(filename);

    return jsonload(nlohmann::json::parse(file));
}

std::string Objects::pb_dumps() const {

    session_proto::Objects proto;
    proto.set_name(name);

    if (has_guid())
        proto.set_guid(guid());

    dump_pb_list(*points, proto.mutable_points());
    dump_pb_list(*lines, proto.mutable_lines());
    dump_pb_list(*planes, proto.mutable_planes());
    dump_pb_list(*bboxes, proto.mutable_bboxes());
    dump_pb_list(*polylines, proto.mutable_polylines());
    dump_pb_list(*pointclouds, proto.mutable_pointclouds());
    dump_pb_list(*meshes, proto.mutable_meshes());
    dump_pb_list(*nurbscurves, proto.mutable_nurbscurves());
    dump_pb_list(*nurbssurfaces, proto.mutable_nurbssurfaces());
    dump_pb_list(*breps, proto.mutable_breps());
    dump_pb_list(*elements, proto.mutable_elements());

    for (const Component& component : *components)
        proto.add_components()->ParseFromString(component.pb_dumps());

    dump_pb_list(*instances, proto.mutable_instances());

    return proto.SerializeAsString();
}

Objects Objects::pb_loads(const std::string& data) {

    session_proto::Objects proto;
    proto.ParseFromString(data);
    Objects objects(proto.name());

    if (!proto.guid().empty())
        objects.guid() = proto.guid();

    load_pb_list(proto.points(), *objects.points);
    load_pb_list(proto.lines(), *objects.lines);
    load_pb_list(proto.planes(), *objects.planes);
    load_pb_list(proto.bboxes(), *objects.bboxes);
    load_pb_list(proto.polylines(), *objects.polylines);
    load_pb_list(proto.pointclouds(), *objects.pointclouds);
    load_pb_list(proto.meshes(), *objects.meshes);
    load_pb_list(proto.nurbscurves(), *objects.nurbscurves);
    load_pb_list(proto.nurbssurfaces(), *objects.nurbssurfaces);
    load_pb_list(proto.breps(), *objects.breps);

    for (const session_proto::Element& element : proto.elements())
        objects.elements->push_back(Element::pb_loads_polymorphic(element.SerializeAsString()));

    for (const session_proto::Component& component : proto.components())
        objects.components->push_back(Component::pb_loads(component.SerializeAsString()));

    load_pb_list(proto.instances(), *objects.instances);

    return objects;
}

void Objects::pb_dump(const std::string& filename) const {
    std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

Objects Objects::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

std::ostream& operator<<(std::ostream& os, const Objects& objects) {
    return os << objects.str();
}
} // namespace session_cpp
