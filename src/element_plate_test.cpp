#include "mini_test.h"
#include "element.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// ElementPlate
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("ElementPlate", "Constructor") {
    // uncomment #include "element_plate.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    std::vector<Point> polygon = {
        Point(0, 0, 0),
        Point(2, 0, 0),
        Point(2, 2, 0),
        Point(0, 2, 0),
    };
    ElementPlate p(polygon, 0.2, "plate1");

    auto& geo = p.geometry();
    std::string name = p.name;
    const std::string& guid = p.guid();
    std::string pstr = p.str();
    std::string prepr = p.repr();

    ElementPlate pcopy = p.duplicate();
    ElementPlate p2(polygon, 0.2, "plate1");
    ElementPlate p3(polygon, 0.5, "plate1");

    MINI_CHECK(name == "plate1");
    MINI_CHECK(!guid.empty());
    MINI_CHECK(std::holds_alternative<Mesh>(geo));
    MINI_CHECK(p.polygon().size() == 4);
    MINI_CHECK(p.thickness() == 0.2);
    MINI_CHECK(pstr == "ElementPlate(plate1, 4 pts, 0.2)");
    MINI_CHECK(prepr == "ElementPlate(" + guid + ", plate1, 4 pts, 0.2)");
    MINI_CHECK(pcopy == p && pcopy.guid() != p.guid());
    MINI_CHECK(p == p2);
    MINI_CHECK(p != p3);
}

MINI_TEST("ElementPlate", "Default Polygon") {
    // uncomment #include "element_plate.h"
    // uncomment #include "mesh.h"
    ElementPlate p;

    MINI_CHECK(std::holds_alternative<Mesh>(p.geometry()));
    MINI_CHECK(p.polygon().size() == 4);
    MINI_CHECK(p.thickness() == 0.1);
}

MINI_TEST("ElementPlate", "Setters") {
    // uncomment #include "element_plate.h"
    // uncomment #include "point.h"
    ElementPlate p;
    p.set_thickness(0.3);
    p.set_polygon({
        Point(0, 0, 0),
        Point(3, 0, 0),
        Point(3, 3, 0),
        Point(0, 3, 0),
    });

    MINI_CHECK(p.thickness() == 0.3);
    MINI_CHECK(p.polygon().size() == 4);
    MINI_CHECK(p.has_geometry());
}

MINI_TEST("ElementPlate", "Mesh Topology") {
    // uncomment #include "element_plate.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    std::vector<Point> polygon = {
        Point(0, 0, 0),
        Point(1, 0, 0),
        Point(1, 1, 0),
        Point(0, 1, 0),
    };
    ElementPlate p(polygon, 0.5);
    auto& geo = std::get<Mesh>(p.geometry());

    MINI_CHECK(geo.vertex.size() == 8);
    MINI_CHECK(geo.face.size() == 6);
}

MINI_TEST("ElementPlate", "Aabb") {
    // uncomment #include "element_plate.h"
    // uncomment #include "obb.h"
    // uncomment #include "point.h"
    std::vector<Point> polygon = {
        Point(0, 0, 0),
        Point(2, 0, 0),
        Point(2, 2, 0),
        Point(0, 2, 0),
    };
    ElementPlate p(polygon, 0.2);
    OBB aabb = p.aabb();

    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(aabb.half_size[2], 0.1));
}

MINI_TEST("ElementPlate", "Compute Point") {
    // uncomment #include "element_plate.h"
    // uncomment #include "point.h"
    std::vector<Point> polygon = {
        Point(0, 0, 0),
        Point(2, 0, 0),
        Point(2, 2, 0),
        Point(0, 2, 0),
    };
    ElementPlate p(polygon, 0.2);
    Point pt = p.point();

    MINI_CHECK(TOLERANCE.is_close(pt[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(pt[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(pt[2], -0.1));
}

MINI_TEST("ElementPlate", "Triangle Polygon") {
    // uncomment #include "element_plate.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    std::vector<Point> polygon = {
        Point(0, 0, 0),
        Point(1, 0, 0),
        Point(0.5, 1, 0),
    };
    ElementPlate p(polygon, 0.1);
    auto& geo = std::get<Mesh>(p.geometry());

    MINI_CHECK(geo.vertex.size() == 6);
    MINI_CHECK(geo.face.size() == 5);
}

MINI_TEST("ElementPlate", "Json Roundtrip") {
    // uncomment #include "element_plate.h"
    // uncomment #include "point.h"
    // uncomment #include "xform.h"
    std::vector<Point> polygon = {
        Point(0, 0, 0),
        Point(2, 0, 0),
        Point(2, 2, 0),
        Point(0, 2, 0),
    };
    ElementPlate p(polygon, 0.3, "json_plate");
    p.session_transformation = Xform::translation(1.0, 2.0, 3.0);

    std::string fname = "serialization/test_plate_element.json";
    p.json_dump(fname);
    ElementPlate loaded = ElementPlate::json_load(fname);

    MINI_CHECK(loaded.name == "json_plate");
    MINI_CHECK(TOLERANCE.is_close(loaded.thickness(), 0.3));
    MINI_CHECK(loaded.polygon().size() == 4);
    MINI_CHECK(TOLERANCE.is_close(loaded.polygon()[1][0], 2.0));
}

MINI_TEST("ElementPlate", "Protobuf Roundtrip") {
    // uncomment #include "element_plate.h"
    // uncomment #include "point.h"
    // uncomment #include "xform.h"
    std::vector<Point> polygon = {
        Point(0, 0, 0),
        Point(2, 0, 0),
        Point(2, 2, 0),
        Point(0, 2, 0),
    };
    ElementPlate p(polygon, 0.3, "proto_plate");
    p.session_transformation = Xform::translation(1.0, 2.0, 3.0);

    std::string path = "serialization/test_plate_element.bin";
    p.pb_dump(path);
    ElementPlate loaded = ElementPlate::pb_load(path);

    MINI_CHECK(loaded.name == "proto_plate");
    MINI_CHECK(TOLERANCE.is_close(loaded.thickness(), 0.3));
    MINI_CHECK(loaded.polygon().size() == 4);
    MINI_CHECK(TOLERANCE.is_close(loaded.polygon()[1][0], 2.0));
}

MINI_TEST("ElementPlate", "From Top Bottom") {
    // uncomment #include "element_plate.h"
    // uncomment #include "point.h"
    std::vector<Point> bottom = {Point(0,0,0), Point(2,0,0), Point(2,2,0), Point(0,2,0), Point(0,0,0)};
    std::vector<Point> top    = {Point(0,0,1), Point(2,0,1), Point(2,2,1), Point(0,2,1), Point(0,0,1)};
    ElementPlate p(bottom, top, "tb_plate");
    MINI_CHECK(p.polygon().size() == 4);
    MINI_CHECK(p.polygon_top().size() == 4);
    MINI_CHECK(TOLERANCE.is_close(p.thickness(), 1.0));
    MINI_CHECK(TOLERANCE.is_close(p.polygon()[0][2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(p.polygon_top()[0][2], 1.0));
    // Reversed argument order should auto-swap
    ElementPlate pr(top, bottom, "tb_plate_r");
    MINI_CHECK(TOLERANCE.is_close(pr.polygon()[0][2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pr.polygon_top()[0][2], 1.0));
}

///////////////////////////////////////////////////////////////////////////////////////////
// ElementPlate - Polylines/Planes/Edge Vectors/Axis/Joinery
///////////////////////////////////////////////////////////////////////////////////////////

MINI_TEST("ElementPlate", "Polylines") {
    std::vector<Point> polygon = {Point(0,0,0), Point(1,0,0), Point(1,1,0), Point(0,1,0)};
    ElementPlate p(polygon, 0.2);
    auto pls = p.polylines();
    MINI_CHECK(pls.size() == 6);
    MINI_CHECK(pls[0].point_count() == 5);
    MINI_CHECK(pls[1].point_count() == 5);
    for (size_t i = 2; i < 6; ++i) MINI_CHECK(pls[i].point_count() == 5);
}

MINI_TEST("ElementPlate", "Planes") {
    std::vector<Point> polygon = {Point(0,0,0), Point(1,0,0), Point(1,1,0), Point(0,1,0)};
    ElementPlate p(polygon, 0.2);
    auto pls = p.planes();
    MINI_CHECK(pls.size() == 6);
    MINI_CHECK(TOLERANCE.is_close(pls[0].z_axis()[2], 1.0));
    MINI_CHECK(TOLERANCE.is_close(pls[1].z_axis()[2], -1.0));
}

MINI_TEST("ElementPlate", "Edge Vectors") {
    std::vector<Point> polygon = {Point(0,0,0), Point(1,0,0), Point(1,1,0), Point(0,1,0)};
    ElementPlate p(polygon, 0.2);
    auto evs = p.edge_vectors();
    MINI_CHECK(evs.size() == 4);
    MINI_CHECK(TOLERANCE.is_close(evs[0][0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(evs[0][1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(evs[1][0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(evs[1][1], 1.0));
}

MINI_TEST("ElementPlate", "Axis") {
    std::vector<Point> polygon = {Point(0,0,0), Point(2,0,0), Point(2,2,0), Point(0,2,0)};
    ElementPlate p(polygon, 0.4);
    auto ax = p.axis();
    MINI_CHECK(ax.has_value());
    MINI_CHECK(TOLERANCE.is_close(ax->start()[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(ax->start()[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(ax->start()[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(ax->end()[2], -0.4));
}

MINI_TEST("ElementPlate", "Joint Types") {
    ElementPlate p;
    MINI_CHECK(p.joint_types().empty());
    p.set_joint_types({1, 2, 3, 4});
    MINI_CHECK(p.joint_types().size() == 4);
    MINI_CHECK(p.joint_types()[0] == 1);
    MINI_CHECK(p.joint_types()[3] == 4);
}

MINI_TEST("ElementPlate", "J Mf") {
    ElementPlate p;
    MINI_CHECK(p.j_mf().empty());
    p.set_j_mf({
        {{0, true, 0.5}, {1, false, 0.3}},
        {},
        {{2, true, 0.8}},
    });
    MINI_CHECK(p.j_mf().size() == 3);
    MINI_CHECK(p.j_mf()[0].size() == 2);
    MINI_CHECK(p.j_mf()[0][0].joint_id == 0);
    MINI_CHECK(p.j_mf()[0][0].is_male == true);
    MINI_CHECK(TOLERANCE.is_close(p.j_mf()[0][0].parameter, 0.5));
    MINI_CHECK(p.j_mf()[2][0].joint_id == 2);
}

MINI_TEST("ElementPlate", "Key") {
    ElementPlate p;
    MINI_CHECK(p.key().empty());
    p.set_key("plate_A");
    MINI_CHECK(p.key() == "plate_A");
}

MINI_TEST("ElementPlate", "Component Plane") {
    ElementPlate p;
    MINI_CHECK(!p.component_plane().has_value());
    p.set_component_plane(Plane(Point(1, 2, 3), Vector(1, 0, 0), Vector(0, 1, 0)));
    MINI_CHECK(TOLERANCE.is_close(p.component_plane()->origin()[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(p.component_plane()->origin()[1], 2.0));
}

MINI_TEST("ElementPlate", "Json Roundtrip Joinery") {
    std::vector<Point> polygon = {Point(0,0,0), Point(2,0,0), Point(2,2,0), Point(0,2,0)};
    ElementPlate p(polygon, 0.3, "joinery_plate");
    p.set_joint_types({1, 2, 3, 4});
    p.set_j_mf({{{0, true, 0.5}}, {}, {{1, false, 0.3}}});
    p.set_key("plate_A");
    p.set_component_plane(Plane(Point(1, 2, 3), Vector(1, 0, 0), Vector(0, 1, 0)));

    std::string fname = "serialization/test_plate_element_joinery.json";
    p.json_dump(fname);
    ElementPlate loaded = ElementPlate::json_load(fname);

    MINI_CHECK(loaded.joint_types().size() == 4);
    MINI_CHECK(loaded.j_mf().size() == 3);
    MINI_CHECK(loaded.key() == "plate_A");
    MINI_CHECK(loaded.component_plane().has_value());
    MINI_CHECK(TOLERANCE.is_close(loaded.component_plane()->origin()[0], 1.0));
}

MINI_TEST("ElementPlate", "Protobuf Roundtrip Joinery") {
    std::vector<Point> polygon = {Point(0,0,0), Point(2,0,0), Point(2,2,0), Point(0,2,0)};
    ElementPlate p(polygon, 0.3, "joinery_plate");
    p.set_joint_types({1, 2, 3, 4});
    p.set_j_mf({{{0, true, 0.5}}, {}, {{1, false, 0.3}}});
    p.set_key("plate_A");
    p.set_component_plane(Plane(Point(1, 2, 3), Vector(1, 0, 0), Vector(0, 1, 0)));

    std::string path = "serialization/test_plate_element_joinery.bin";
    p.pb_dump(path);
    ElementPlate loaded = ElementPlate::pb_load(path);

    MINI_CHECK(loaded.joint_types().size() == 4);
    MINI_CHECK(loaded.j_mf().size() == 3);
    MINI_CHECK(loaded.j_mf()[0][0].joint_id == 0);
    MINI_CHECK(loaded.j_mf()[0][0].is_male == true);
    MINI_CHECK(loaded.key() == "plate_A");
    MINI_CHECK(loaded.component_plane().has_value());
    MINI_CHECK(TOLERANCE.is_close(loaded.component_plane()->origin()[0], 1.0));
}

} // namespace session_cpp
