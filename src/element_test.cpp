#include "mini_test.h"
#include "element.h"
#include "element.pb.h"
#include "tolerance.h"

#include <cmath>
#include <memory>

using namespace session_cpp::mini_test;

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Element
// ═══════════════════════════════════════════════════════════════════════════

MINI_TEST("Element", "Constructor") {
    Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m, "test_element");

    auto& geo = e.geometry();
    std::string name = e.name;
    const std::string& guid = e.guid();
    bool dirty = e.is_dirty();

    std::string estr = e.str();
    std::string erepr = e.repr();

    Element ecopy = e.duplicate();

    Element e2(Mesh(), "test_element");
    Element e3(BRep(), "other");

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
    Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    Xform xf = Xform::translation(10.0, 20.0, 30.0);
    e.place(xf);

    MINI_CHECK(e.is_dirty());
    if (auto* mesh = std::get_if<Mesh>(&e.geometry())) {
        double min_x = std::numeric_limits<double>::max();
        for (const auto& [k, v] : mesh->vertex) min_x = std::min(min_x, v.x);
        MINI_CHECK(min_x > 9.0);
    }
}

MINI_TEST("Element", "Add Geometry Op") {
    Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);

    auto my_feature = [](Mesh geo) -> Mesh { return geo; };
    e.add_geometry_op(my_feature);

    Element eb(BRep::create_box(1.0, 1.0, 1.0), "brep_feature");
    eb.add_geometry_op([](Mesh) -> Mesh { return Mesh(); });
    auto sg = eb.session_geometry(Xform::identity());

    MINI_CHECK(e.is_dirty());
    MINI_CHECK(e.geometry_ops_count() == 1);
    MINI_CHECK(std::holds_alternative<BRep>(sg));
}

MINI_TEST("Element", "AABB") {
    Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    OBB aabb = e.aabb();

    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[0], 0.5));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[1], 0.5));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[2], 0.0));
}

MINI_TEST("Element", "OBB") {
    Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    OBB obb = e.obb();

    MINI_CHECK(TOLERANCE.is_close(obb.half_size[0], 0.5));
    MINI_CHECK(TOLERANCE.is_close(obb.half_size[1], 0.5));
}

MINI_TEST("Element", "Session Geometry") {
    Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    Xform e_xf = Xform::translation(10.0, 0.0, 0.0);
    auto sg = e.session_geometry(e_xf);

    MINI_CHECK(std::holds_alternative<Mesh>(sg));
    auto& mesh = std::get<Mesh>(sg);
    MINI_CHECK(TOLERANCE.is_close(mesh.vertex.at(0).x, 10.0));
    MINI_CHECK(TOLERANCE.is_close(mesh.vertex.at(1).x, 11.0));
    MINI_CHECK(&std::get<Mesh>(e.geometry()) != &mesh);
}

MINI_TEST("Element", "Reset") {
    Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(2, 0, 0),
            Point(2, 2, 0),
            Point(0, 2, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    auto _ = e.aabb();
    auto _2 = e.point();
    e.reset();

    MINI_CHECK(e.is_dirty());
    MINI_CHECK(!e.cached_aabb().has_value());
    MINI_CHECK(!e.cached_obb().has_value());
    MINI_CHECK(!e.cached_collision_mesh().has_value());
    MINI_CHECK(!e.cached_point().has_value());
}

MINI_TEST("Element", "Compute Point") {
    Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(2, 0, 0),
            Point(2, 2, 0),
            Point(0, 2, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m);
    Point pt = e.point();

    MINI_CHECK(TOLERANCE.is_close(pt[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(pt[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(pt[2], 0.0));
}

MINI_TEST("Element", "Brep Aabb") {
    BRep b = BRep::create_box(2.0, 3.0, 4.0);
    Element e(b, "brep_element");
    OBB aabb = e.aabb();
    Point pt = e.point();

    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[1], 1.5));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[2], 2.0));
    MINI_CHECK(TOLERANCE.is_close(pt[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pt[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pt[2], 0.0));
}

MINI_TEST("Element", "Json Roundtrip") {
    Mesh m = Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        },
        {{0, 1, 2, 3}}
    );
    Element e(m, "json_test");

    std::string fname = "serialization/test_element.json";
    e.file_json_dump(fname);
    Element loaded = Element::file_json_load(fname);

    MINI_CHECK(loaded.name == "json_test");
    MINI_CHECK(std::holds_alternative<Mesh>(loaded.geometry()));
    MINI_CHECK(std::get<Mesh>(loaded.geometry()).vertex.size() == 4);
}

MINI_TEST("Element", "Protobuf Roundtrip") {
    BRep b = BRep::create_box(2.0, 3.0, 4.0);
    Element e(b, "proto_test");

    std::string path = "serialization/test_element.bin";
    e.pb_dump(path);
    Element loaded = Element::pb_load(path);

    MINI_CHECK(loaded.name == "proto_test");
    MINI_CHECK(std::holds_alternative<BRep>(loaded.geometry()));
    MINI_CHECK(std::get<BRep>(loaded.geometry()).face_count() == 6);
    MINI_CHECK(std::get<BRep>(loaded.geometry()).vertex_count() == 8);
}

// ═══════════════════════════════════════════════════════════════════════════
// Element - Polylines
// ═══════════════════════════════════════════════════════════════════════════

MINI_TEST("Element", "Polylines") {
    Mesh m = Mesh::from_vertices_and_faces(
        {Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0), Point(0, 1, 0)},
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
    MINI_CHECK(std::fabs(normal[0]) < 1e-12 && std::fabs(normal[1]) < 1e-12 && normal[2] > 0.0);
    MINI_CHECK(e.edge_vectors().empty());
    MINI_CHECK(!e.axis().has_value());
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
    TestPlate() = default;
    TestPlate(const Mesh& geo, const std::string& n, double thickness, std::vector<int> codes)
        : Element(geo, n), thickness(thickness), codes(std::move(codes)) {}

    double thickness = 0.0;
    std::vector<int> codes;

    std::string element_type_name() const override { return "TestPlate"; }

    std::string element_data_dumps() const override {
        std::string out = std::to_string(thickness);
        for (int c : codes) out += "," + std::to_string(c);
        return out;
    }

    static void register_with_kernel() {
        Element::register_type("TestPlate", [](const std::string& data) {
            Element base = Element::pb_loads(data);
            session_proto::Element proto;
            proto.ParseFromString(data);

            auto plate = std::make_shared<TestPlate>();
            static_cast<Element&>(*plate) = base;
            plate->guid() = proto.guid();

            std::string payload = proto.element_data();
            size_t pos = payload.find(',');
            plate->thickness = std::stod(payload.substr(0, pos));
            while (pos != std::string::npos) {
                size_t next = payload.find(',', pos + 1);
                plate->codes.push_back(std::stoi(payload.substr(pos + 1, next - pos - 1)));
                pos = next;
            }
            return plate;
        });
    }
};

Mesh unit_quad() {
    return Mesh::from_vertices_and_faces(
        {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0)
        },
        {{0, 1, 2, 3}}
    );
}

}  // namespace

MINI_TEST("Element", "Registry Round Trip") {
    TestPlate::register_with_kernel();
    MINI_CHECK(Element::is_registered("TestPlate"));

    TestPlate plate(unit_quad(), "plate_0", 12.5, {30, 11, 20});
    const std::string guid = plate.guid();
    auto loaded = Element::pb_loads_polymorphic(plate.pb_dumps());

    auto* as_plate = dynamic_cast<TestPlate*>(loaded.get());
    MINI_CHECK(as_plate != nullptr);
    MINI_CHECK(as_plate->element_type_name() == "TestPlate");

    MINI_CHECK(as_plate->guid() == guid);
    MINI_CHECK(as_plate->name == "plate_0");
    MINI_CHECK(std::holds_alternative<Mesh>(as_plate->geometry()));
    MINI_CHECK(std::abs(as_plate->thickness - 12.5) < 1e-9);
    MINI_CHECK(as_plate->codes.size() == 3);
    MINI_CHECK(as_plate->codes[0] == 30 && as_plate->codes[1] == 11 && as_plate->codes[2] == 20);
}

MINI_TEST("Element", "Registry Unknown Type Degrades") {
    MINI_CHECK(!Element::is_registered("NeverRegistered"));

    session_proto::Element proto;
    proto.ParseFromString(Element(unit_quad(), "mystery").pb_dumps());
    proto.set_element_type("NeverRegistered");
    proto.set_element_data("whatever this package meant");

    auto loaded = Element::pb_loads_polymorphic(proto.SerializeAsString());
    MINI_CHECK(loaded != nullptr);
    MINI_CHECK(loaded->name == "mystery");
    MINI_CHECK(std::holds_alternative<Mesh>(loaded->geometry()));
}

MINI_TEST("Element", "Features Round Trip") {
    Element e(unit_quad(), "plate_0");
    e.set_insertion_vectors({Vector(0, 0, 1), Vector(1, 0, 0)});
    e.set_dimensions(Vector(120.0, 80.0, 12.5));
    e.add_feature(ElementFeature("cut", 2,
        {Polyline({Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0), Point(0, 0, 0)})}, "notch"));
    std::string feature_guid = e.features()[0].guid();

    Element loaded = Element::pb_loads(e.pb_dumps());

    MINI_CHECK(loaded.insertion_vectors().size() == 2);
    MINI_CHECK(loaded.insertion_vectors()[0] == Vector(0, 0, 1));
    MINI_CHECK(loaded.dimensions().has_value());
    MINI_CHECK(std::abs((*loaded.dimensions())[2] - 12.5) < 1e-9);
    MINI_CHECK(loaded.features().size() == 1);
    MINI_CHECK(loaded.features()[0].feature_type == "cut");
    MINI_CHECK(loaded.features()[0].face_index == 2);
    MINI_CHECK(loaded.features()[0].name == "notch");
    MINI_CHECK(loaded.features()[0].outlines.size() == 1);
    MINI_CHECK(loaded.features()[0].guid() == feature_guid);
}

MINI_TEST("Element", "Dimensions Are Nominal Not Measured") {
    Element e(unit_quad(), "plate");
    MINI_CHECK(!e.dimensions().has_value());

    e.set_dimensions(Vector(120.0, 80.0, 12.5));
    OBB measured = e.obb();

    MINI_CHECK(std::abs((*e.dimensions())[0] - 120.0) < 1e-9);
    MINI_CHECK(measured.half_size[0] < 1.0);
}

MINI_TEST("Element", "Registry Leaves Base Bytes Unchanged") {
    Element e(unit_quad(), "plain");
    session_proto::Element proto;
    proto.ParseFromString(e.pb_dumps());

    MINI_CHECK(proto.element_type().empty());
    MINI_CHECK(proto.element_data().empty());
    MINI_CHECK(e.element_type_name().empty());
}

MINI_TEST("Element", "Registry Json Round Trip") {
    TestPlate::register_with_kernel();

    TestPlate plate(unit_quad(), "plate_json", 9.5, {7, 8});
    auto loaded = Element::file_json_loads_polymorphic(plate.file_json_dumps());

    auto* as_plate = dynamic_cast<TestPlate*>(loaded.get());
    MINI_CHECK(as_plate != nullptr);
    MINI_CHECK(as_plate->name == "plate_json");
    MINI_CHECK(as_plate->guid() == plate.guid());
    MINI_CHECK(std::abs(as_plate->thickness - 9.5) < 1e-9);
    MINI_CHECK(as_plate->codes.size() == 2);
    MINI_CHECK(as_plate->codes[0] == 7 && as_plate->codes[1] == 8);
}

MINI_TEST("Element", "Throwing Factory Degrades To Base") {
    Element::register_type("Exploding", [](const std::string&) -> std::shared_ptr<Element> {
        throw std::runtime_error("this package is broken");
    });

    session_proto::Element proto;
    proto.ParseFromString(Element(unit_quad(), "victim").pb_dumps());
    proto.set_element_type("Exploding");

    auto loaded = Element::pb_loads_polymorphic(proto.SerializeAsString());
    MINI_CHECK(loaded != nullptr);
    MINI_CHECK(loaded->name == "victim");
    MINI_CHECK(std::holds_alternative<Mesh>(loaded->geometry()));
}

MINI_TEST("Element", "Unknown Type Survives Resave") {
    session_proto::Element proto;
    proto.ParseFromString(Element(unit_quad(), "plate").pb_dumps());
    proto.set_element_type("wood::Plate");
    proto.set_element_data("the package's own bytes");
    std::string original = proto.SerializeAsString();

    Element loaded = Element::pb_loads(original);
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

    Element copy = e.duplicate();

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
    Polyline outline({
        Point(0, 0, 0),
        Point(1, 0, 0),
        Point(1, 1, 0),
        Point(0, 0, 0)
    });
    ElementFeature f("cut", 2, {outline}, "notch");

    MINI_CHECK(f.feature_type == "cut");
    MINI_CHECK(f.face_index == 2);
    MINI_CHECK(f.name == "notch");
    MINI_CHECK(f.outlines.size() == 1);

    ElementFeature same("cut", 2, {outline}, "notch");
    MINI_CHECK(f == same);
    MINI_CHECK(!(f != same));
    MINI_CHECK(f.guid() != same.guid());

    ElementFeature other("drill", 2, {outline}, "notch");
    MINI_CHECK(f != other);

    MINI_CHECK(f.str() == "ElementFeature(cut, face 2, 1 outline(s))");
    MINI_CHECK(f.repr() == f.str());

    ElementFeature empty;
    MINI_CHECK(empty.face_index == -1);
    MINI_CHECK(empty.outlines.empty());
}

MINI_TEST("ElementFeature", "Json Roundtrip") {
    ElementFeature f("cut", 2,
        {Polyline({Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0), Point(0, 0, 0)})}, "notch");
    std::string feature_guid = f.guid();

    std::string fname = "serialization/test_element_feature.json";
    f.file_json_dump(fname);
    ElementFeature loaded = ElementFeature::file_json_load(fname);

    MINI_CHECK(loaded == f);
    MINI_CHECK(loaded.outlines.size() == 1);
    MINI_CHECK(loaded.guid() == feature_guid);
}

MINI_TEST("ElementFeature", "Protobuf Roundtrip") {
    ElementFeature f("drill", 5,
        {Polyline({Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0), Point(0, 0, 0)})}, "hole");
    std::string feature_guid = f.guid();

    std::string path = "serialization/test_element_feature.bin";
    f.pb_dump(path);
    ElementFeature loaded = ElementFeature::pb_load(path);

    MINI_CHECK(loaded == f);
    MINI_CHECK(loaded.feature_type == "drill");
    MINI_CHECK(loaded.face_index == 5);
    MINI_CHECK(loaded.outlines.size() == 1);
    MINI_CHECK(loaded.guid() == feature_guid);
}

} // namespace session_cpp
