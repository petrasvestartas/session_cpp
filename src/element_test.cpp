#include "mini_test.h"
#include "element.h"
#include "element.pb.h"
#include "tolerance.h"

#include <limits>
#include <memory>
#include <stdexcept>

using namespace session_cpp::mini_test;

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Element
// ═══════════════════════════════════════════════════════════════════════════

namespace {

/// Geometry op that returns the mesh unchanged.
Mesh my_feature(Mesh geo) {
    return geo;
}

/// Geometry op that drops the mesh.
Mesh empty_mesh(Mesh) {
    return Mesh();
}

} // namespace

MINI_TEST("Element", "Constructor") {

    const Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m, "test_element");

    const ElementGeometry& geo = e.geometry();
    const std::string name = e.name;
    const std::string& guid = e.guid();
    const bool dirty = e.is_dirty();

    const std::string estr = e.str();
    const std::string erepr = e.repr();

    const Element ecopy = e.duplicate();

    const Element e2(Mesh(), "test_element");
    const Element e3(BRep(), "other");

    MINI_CHECK(name == "test_element");
    MINI_CHECK(!guid.empty());
    MINI_CHECK(dirty);
    MINI_CHECK(std::holds_alternative<Mesh>(geo));
    MINI_CHECK(estr == "Element(test_element, Mesh)");
    MINI_CHECK(erepr == "Element(" + guid + ", test_element, Mesh)");
    MINI_CHECK(ecopy == e && ecopy.guid() != e.guid());
    MINI_CHECK(e == e2);
    MINI_CHECK(e != e3);
}

MINI_TEST("Element", "Place") {

    const Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    const Xform xf = Xform::translation(10.0, 20.0, 30.0);
    e.place(xf);

    MINI_CHECK(e.is_dirty());

    if (const Mesh* mesh = std::get_if<Mesh>(&e.geometry())) {
        double min_x = std::numeric_limits<double>::max();

        for (const std::pair<const size_t, VertexData>& entry : mesh->vertex)
            min_x = std::min(min_x, entry.second.position()[0]);

        MINI_CHECK(min_x > 9.0);
    }
}

MINI_TEST("Element", "Place Moves Features") {

    const Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    e.add_feature(ElementFeature("contact", 0, {Polyline(std::vector<Point>{Point(0, 0, 0), Point(1, 0, 0)})}));
    e.set_insertion_vectors({Vector(1, 0, 0)});
    const std::string guid = e.features()[0].guid();
    e.place(Xform::translation(0.0, 0.0, 5.0) * Xform::rotation_z(Tolerance::PI / 2.0));

    const Point moved = e.features()[0].outlines[0].get_point(1);
    const Vector turned = e.insertion_vectors()[0];

    MINI_CHECK(TOLERANCE.is_close(moved[0], 0.0) && TOLERANCE.is_close(moved[1], 1.0) && TOLERANCE.is_close(moved[2], 5.0));
    MINI_CHECK(TOLERANCE.is_close(turned[0], 0.0) && TOLERANCE.is_close(turned[1], 1.0) && TOLERANCE.is_close(turned[2], 0.0));
    MINI_CHECK(e.features()[0].guid() == guid);
}

MINI_TEST("Element", "Add Geometry Op") {

    const Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    e.add_geometry_op(my_feature);

    Element eb(BRep::create_box(1.0, 1.0, 1.0), "brep_feature");
    eb.add_geometry_op(empty_mesh);
    const ElementGeometry sg = eb.session_geometry(Xform::identity());

    MINI_CHECK(e.is_dirty());
    MINI_CHECK(e.geometry_ops_count() == 1);
    MINI_CHECK(std::holds_alternative<BRep>(sg));
}

MINI_TEST("Element", "AABB") {

    const Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    const OBB aabb = e.aabb();

    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[0], 0.5));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[1], 0.5));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[2], 0.0));
    MINI_CHECK(!e.is_dirty());

    e.add_geometry_op(my_feature);

    MINI_CHECK(e.is_dirty());
    MINI_CHECK(!e.cached_aabb().has_value());
}

MINI_TEST("Element", "OBB") {

    const Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    const OBB obb = e.obb();

    MINI_CHECK(TOLERANCE.is_close(obb.half_size[0], 0.5));
    MINI_CHECK(TOLERANCE.is_close(obb.half_size[1], 0.5));
}

MINI_TEST("Element", "Session Geometry") {

    const Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    const Element e(m);
    const Xform e_xf = Xform::translation(10.0, 0.0, 0.0);
    const ElementGeometry sg = e.session_geometry(e_xf);

    MINI_CHECK(std::holds_alternative<Mesh>(sg));

    const Mesh& mesh = std::get<Mesh>(sg);

    MINI_CHECK(TOLERANCE.is_close(mesh.vertex.at(0).position()[0], 10.0));
    MINI_CHECK(TOLERANCE.is_close(mesh.vertex.at(1).position()[0], 11.0));
    MINI_CHECK(&std::get<Mesh>(e.geometry()) != &mesh);
}

MINI_TEST("Element", "Reset") {

    const Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(2, 0, 0),
            Point(2, 2, 0),
            Point(0, 2, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    e.aabb();
    e.point();
    e.reset();

    MINI_CHECK(e.is_dirty());
    MINI_CHECK(!e.cached_aabb().has_value());
    MINI_CHECK(!e.cached_obb().has_value());
    MINI_CHECK(!e.cached_collision_mesh().has_value());
    MINI_CHECK(!e.cached_point().has_value());
}

MINI_TEST("Element", "Compute Point") {

    const Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(2, 0, 0),
            Point(2, 2, 0),
            Point(0, 2, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    const Point pt = e.point();

    MINI_CHECK(TOLERANCE.is_close(pt[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(pt[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(pt[2], 0.0));
}

MINI_TEST("Element", "Brep Aabb") {

    const BRep b = BRep::create_box(2.0, 3.0, 4.0);
    Element e(b, "brep_element");
    const OBB aabb = e.aabb();
    const Point pt = e.point();

    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[1], 1.5));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[2], 2.0));
    MINI_CHECK(TOLERANCE.is_close(pt[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pt[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pt[2], 0.0));
}

MINI_TEST("Element", "Json Roundtrip") {

    const Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    const Element e(m, "json_test");

    const std::string fname = "serialization/test_element.json";
    e.file_json_dump(fname);
    const Element loaded = Element::file_json_load(fname);

    MINI_CHECK(loaded.name == "json_test");
    MINI_CHECK(std::holds_alternative<Mesh>(loaded.geometry()));
    MINI_CHECK(std::get<Mesh>(loaded.geometry()).vertex.size() == 4);
}

MINI_TEST("Element", "Protobuf Roundtrip") {

    const BRep b = BRep::create_box(2.0, 3.0, 4.0);
    const Element e(b, "proto_test");

    const std::string path = "serialization/test_element.bin";
    e.pb_dump(path);
    const Element loaded = Element::pb_load(path);

    MINI_CHECK(loaded.name == "proto_test");
    MINI_CHECK(std::holds_alternative<BRep>(loaded.geometry()));
    MINI_CHECK(std::get<BRep>(loaded.geometry()).face_count() == 6);
    MINI_CHECK(std::get<BRep>(loaded.geometry()).vertex_count() == 8);
}

// ═══════════════════════════════════════════════════════════════════════════
// Element - Polylines
// ═══════════════════════════════════════════════════════════════════════════

MINI_TEST("Element", "Polylines") {

    const Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m, "test_element");

    MINI_CHECK(e.polylines().size() == 1);
    MINI_CHECK(e.polylines()[0].point_count() == 5);
    MINI_CHECK(e.polylines()[0].get_point(0) == Point(0, 0, 0));
    MINI_CHECK(e.polylines()[0].get_point(4) == Point(0, 0, 0));
    MINI_CHECK(e.planes().size() == 1);
    MINI_CHECK(e.planes()[0].origin() == Point(0.5, 0.5, 0.0));

    const Vector normal = e.planes()[0].z_axis();

    MINI_CHECK(TOLERANCE.is_close(normal[0], 0.0) && TOLERANCE.is_close(normal[1], 0.0) && normal[2] > 0.0);
    MINI_CHECK(e.edge_vectors().empty());
    MINI_CHECK(!e.axis().has_value());
}

MINI_TEST("Element", "Set Polylines Sticks") {

    const Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    e.set_polylines({Polyline(std::vector<Point>{Point(0, 0, 0), Point(2, 0, 0)})});
    e.set_planes({Plane::xy_plane()});

    MINI_CHECK(e.polylines().size() == 1);
    MINI_CHECK(e.polylines()[0].point_count() == 2);
    MINI_CHECK(e.planes()[0].origin() == Point(0, 0, 0));
}

MINI_TEST("Element", "Polylines Empty Without Mesh") {

    MINI_CHECK(Element("no_geometry").polylines().empty());
    MINI_CHECK(Element("no_geometry").planes().empty());
}

// ═══════════════════════════════════════════════════════════════════════════
// Element - Polymorphic registry
// ═══════════════════════════════════════════════════════════════════════════

namespace {

/// Stand-in for a domain element: carries state the kernel knows nothing about.
class TestPlate : public Element {
public:
    double thickness = 0.0; // Plate thickness.
    std::vector<int> codes; // Package codes.

    /// Construct an empty plate.
    TestPlate() = default;

    /// Construct from geometry, name, thickness and codes.
    TestPlate(const Mesh& geo, const std::string& n, double thickness, std::vector<int> codes)
        : Element(geo, n), thickness(thickness), codes(std::move(codes)) {}

    /// Return the registered type name.
    std::string element_type_name() const override {
        return "TestPlate";
    }

    /// Return a copy that keeps the thickness and codes.
    std::shared_ptr<Element> clone() const override {
        return std::make_shared<TestPlate>(*this);
    }

    /// Return the thickness and codes as comma-separated text.
    std::string element_data_dumps() const override {

        std::string out = std::to_string(thickness);

        for (int c : codes)
            out += "," + std::to_string(c);

        return out;
    }

    /// Build a plate from full serialized session_proto.Element bytes.
    static std::shared_ptr<Element> factory(const std::string& data) {

        const Element base = Element::pb_loads(data);
        std::shared_ptr<TestPlate> plate = std::make_shared<TestPlate>();
        static_cast<Element&>(*plate) = base;
        plate->guid() = base.guid();

        const std::string payload = base.element_data_dumps();
        size_t pos = payload.find(',');
        plate->thickness = std::stod(payload.substr(0, pos));

        while (pos != std::string::npos) {
            const size_t next = payload.find(',', pos + 1);
            plate->codes.push_back(std::stoi(payload.substr(pos + 1, next - pos - 1)));
            pos = next;
        }

        return plate;
    }

    /// Register the factory under "TestPlate".
    static void register_with_kernel() {
        Element::register_type("TestPlate", factory);
    }
};

/// Return a unit square mesh in the xy plane.
Mesh unit_quad() {

    return Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
}

/// Stand-in for a broken package: its factory always throws.
std::shared_ptr<Element> explode(const std::string&) {
    throw std::runtime_error("this package is broken");
}

} // namespace

MINI_TEST("Element", "Registry Round Trip") {

    TestPlate::register_with_kernel();

    MINI_CHECK(Element::is_registered("TestPlate"));

    const TestPlate plate(unit_quad(), "plate_0", 12.5, {30, 11, 20});
    const std::string guid = plate.guid();
    const std::shared_ptr<Element> loaded = Element::pb_loads_polymorphic(plate.pb_dumps());
    const std::shared_ptr<Element> copy = plate.clone();

    const TestPlate* as_plate = dynamic_cast<const TestPlate*>(loaded.get());

    MINI_CHECK(as_plate != nullptr);
    MINI_CHECK(as_plate->element_type_name() == "TestPlate");

    MINI_CHECK(as_plate->guid() == guid);
    MINI_CHECK(as_plate->name == "plate_0");
    MINI_CHECK(std::holds_alternative<Mesh>(as_plate->geometry()));
    MINI_CHECK(TOLERANCE.is_close(as_plate->thickness, 12.5));
    MINI_CHECK(as_plate->codes.size() == 3);
    MINI_CHECK(as_plate->codes[0] == 30 && as_plate->codes[1] == 11 && as_plate->codes[2] == 20);
    MINI_CHECK(dynamic_cast<const TestPlate*>(copy.get())->codes == plate.codes);
}

MINI_TEST("Element", "Registry Unknown Type Degrades") {

    MINI_CHECK(!Element::is_registered("NeverRegistered"));

    session_proto::Element proto;
    proto.ParseFromString(Element(unit_quad(), "mystery").pb_dumps());
    proto.set_element_type("NeverRegistered");
    proto.set_element_data("whatever this package meant");

    const std::shared_ptr<Element> loaded = Element::pb_loads_polymorphic(proto.SerializeAsString());

    MINI_CHECK(loaded != nullptr);
    MINI_CHECK(loaded->name == "mystery");
    MINI_CHECK(std::holds_alternative<Mesh>(loaded->geometry()));
}

MINI_TEST("Element", "Features Round Trip") {

    Element e(unit_quad(), "plate_0");
    e.set_insertion_vectors({Vector(0, 0, 1), Vector(1, 0, 0)});
    e.set_dimensions(Vector(120.0, 80.0, 12.5));
    e.add_feature(
        ElementFeature("cut", 2, {Polyline({Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0), Point(0, 0, 0)})}, "notch")
    );
    const std::string feature_guid = e.features()[0].guid();

    const Element loaded = Element::pb_loads(e.pb_dumps());

    MINI_CHECK(loaded.insertion_vectors().size() == 2);
    MINI_CHECK(loaded.insertion_vectors()[0] == Vector(0, 0, 1));
    MINI_CHECK(loaded.dimensions().has_value());
    MINI_CHECK(TOLERANCE.is_close((*loaded.dimensions())[2], 12.5));
    MINI_CHECK(loaded.features().size() == 1);
    MINI_CHECK(loaded.features()[0].feature_type == "cut");
    MINI_CHECK(loaded.features()[0].face_index == 2);
    MINI_CHECK(loaded.features()[0].name == "notch");
    MINI_CHECK(loaded.features()[0].outlines.size() == 1);
    MINI_CHECK(loaded.features()[0].visible);
    MINI_CHECK(loaded.features()[0].guid() == feature_guid);
}

MINI_TEST("Element", "Dimensions Are Nominal Not Measured") {

    Element e(unit_quad(), "plate");

    MINI_CHECK(!e.dimensions().has_value());

    e.set_dimensions(Vector(120.0, 80.0, 12.5));
    const OBB measured = e.obb();

    MINI_CHECK(TOLERANCE.is_close((*e.dimensions())[0], 120.0));
    MINI_CHECK(measured.half_size[0] < 1.0);
}

MINI_TEST("Element", "Registry Leaves Base Bytes Unchanged") {

    const Element e(unit_quad(), "plain");
    session_proto::Element proto;
    proto.ParseFromString(e.pb_dumps());

    MINI_CHECK(proto.element_type().empty());
    MINI_CHECK(proto.element_data().empty());
    MINI_CHECK(e.element_type_name().empty());
}

MINI_TEST("Element", "Registry Json Round Trip") {

    TestPlate::register_with_kernel();

    const TestPlate plate(unit_quad(), "plate_json", 9.5, {7, 8});
    const std::shared_ptr<Element> loaded = Element::file_json_loads_polymorphic(plate.file_json_dumps());

    const TestPlate* as_plate = dynamic_cast<const TestPlate*>(loaded.get());

    MINI_CHECK(as_plate != nullptr);
    MINI_CHECK(as_plate->name == "plate_json");
    MINI_CHECK(as_plate->guid() == plate.guid());
    MINI_CHECK(TOLERANCE.is_close(as_plate->thickness, 9.5));
    MINI_CHECK(as_plate->codes.size() == 2);
    MINI_CHECK(as_plate->codes[0] == 7 && as_plate->codes[1] == 8);
}

MINI_TEST("Element", "Throwing Factory Degrades To Base") {

    Element::register_type("Exploding", explode);

    session_proto::Element proto;
    proto.ParseFromString(Element(unit_quad(), "victim").pb_dumps());
    proto.set_element_type("Exploding");

    const std::shared_ptr<Element> loaded = Element::pb_loads_polymorphic(proto.SerializeAsString());

    MINI_CHECK(loaded != nullptr);
    MINI_CHECK(loaded->name == "victim");
    MINI_CHECK(std::holds_alternative<Mesh>(loaded->geometry()));
}

MINI_TEST("Element", "Unknown Type Survives Resave") {

    session_proto::Element proto;
    proto.ParseFromString(Element(unit_quad(), "plate").pb_dumps());
    proto.set_element_type("wood::Plate");
    proto.set_element_data("the package's own bytes");
    const std::string original = proto.SerializeAsString();

    const Element loaded = Element::pb_loads(original);

    MINI_CHECK(loaded.element_type_name() == "wood::Plate");
    MINI_CHECK(loaded.element_data_dumps() == "the package's own bytes");

    session_proto::Element resaved;
    resaved.ParseFromString(loaded.pb_dumps());

    MINI_CHECK(resaved.element_type() == "wood::Plate");
    MINI_CHECK(resaved.element_data() == "the package's own bytes");
}

MINI_TEST("Element", "Duplicate Keeps Every Field") {

    Element e(unit_quad(), "original");
    e.set_insertion_vectors({Vector(0, 0, 1)});
    e.set_dimensions(Vector(120.0, 80.0, 12.5));
    e.add_feature(ElementFeature("cut", 2, {}, "notch"));

    const Element copy = e.duplicate();

    MINI_CHECK(copy == e);
    MINI_CHECK(copy.guid() != e.guid());
    MINI_CHECK(copy.insertion_vectors().size() == 1);
    MINI_CHECK(copy.dimensions().has_value());
    MINI_CHECK(copy.features().size() == 1);
}

MINI_TEST("Element", "Equality Compares Carried Fields") {

    Element a(unit_quad(), "same");
    Element b(unit_quad(), "same");

    MINI_CHECK(a == b);

    b.set_dimensions(Vector(1, 2, 3));

    MINI_CHECK(a != b);
}

// ═══════════════════════════════════════════════════════════════════════════
// ElementFeature
// ═══════════════════════════════════════════════════════════════════════════

MINI_TEST("ElementFeature", "Constructor") {

    const Polyline outline({Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0), Point(0, 0, 0)});
    const ElementFeature f("cut", 2, {outline}, "notch");

    MINI_CHECK(f.feature_type == "cut");
    MINI_CHECK(f.face_index == 2);
    MINI_CHECK(f.name == "notch");
    MINI_CHECK(f.outlines.size() == 1);
    MINI_CHECK(f.visible);

    const ElementFeature same("cut", 2, {outline}, "notch");

    MINI_CHECK(f == same);
    MINI_CHECK(!(f != same));
    MINI_CHECK(f.guid() != same.guid());

    const ElementFeature other("drill", 2, {outline}, "notch");

    MINI_CHECK(f != other);

    MINI_CHECK(f.str() == "ElementFeature(cut, face 2, 1 outline(s))");
    MINI_CHECK(f.repr() == f.str());

    const ElementFeature empty;

    MINI_CHECK(empty.face_index == -1);
    MINI_CHECK(empty.outlines.empty());
}

MINI_TEST("ElementFeature", "Json Roundtrip") {

    ElementFeature f("cut", 2, {Polyline({Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0), Point(0, 0, 0)})}, "notch");
    f.visible = false;

    const std::string feature_guid = f.guid();

    const std::string fname = "serialization/test_element_feature.json";
    f.file_json_dump(fname);
    const ElementFeature loaded = ElementFeature::file_json_load(fname);

    MINI_CHECK(loaded == f);
    MINI_CHECK(loaded.outlines.size() == 1);
    MINI_CHECK(!loaded.visible);
    MINI_CHECK(loaded.guid() == feature_guid);
}

MINI_TEST("ElementFeature", "Protobuf Roundtrip") {

    ElementFeature f("drill", 5, {Polyline({Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0), Point(0, 0, 0)})}, "hole");
    f.visible = false;

    const std::string feature_guid = f.guid();

    const std::string path = "serialization/test_element_feature.bin";
    f.pb_dump(path);
    const ElementFeature loaded = ElementFeature::pb_load(path);

    MINI_CHECK(loaded == f);
    MINI_CHECK(loaded.feature_type == "drill");
    MINI_CHECK(loaded.face_index == 5);
    MINI_CHECK(loaded.outlines.size() == 1);
    MINI_CHECK(!loaded.visible);
    MINI_CHECK(loaded.guid() == feature_guid);
}

} // namespace session_cpp
