#include "mini_test.h"
#include "instance_ref.h"
#include "instance_ref.pb.h"
#include "tolerance.h"
#include "xform.h"
#include <string>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("InstanceRef", "Constructor") {
        // using session_cpp::ElementFeature;
        // using session_cpp::InstanceRef;
        // using session_cpp::Point;
        // using session_cpp::Polyline;
        // using session_cpp::Xform;

        const Xform x = Xform::translation(10.0, 20.0, 30.0);
        const InstanceRef inst("def-123", x);

        InstanceRef instset = inst;
        instset[0] = 2.0;
        const double m0 = instset[0];

        const std::string istr = inst.str();
        const std::string irepr = inst.repr();

        const InstanceRef instcopy = inst;
        const InstanceRef instother("def-123", x);
        const InstanceRef named = InstanceRef::with_name("custom", "def-9", Xform::identity());

        InstanceRef featured("def-123", x);
        featured.features.push_back(ElementFeature("contact", 0, {Polyline(std::vector<Point>{Point(0, 0, 0), Point(1, 0, 0)})}));
        const InstanceRef featuredcopy = featured;

        MINI_CHECK(inst.name == "my_instance_ref");
        MINI_CHECK(inst.definition_guid == "def-123");
        MINI_CHECK(!inst.guid().empty());
        MINI_CHECK(m0 == 2.0);
        MINI_CHECK(inst[12] == 10.0 && inst[13] == 20.0 && inst[14] == 30.0);
        MINI_CHECK(istr.find("def-123") != std::string::npos);
        MINI_CHECK(irepr.find("InstanceRef") != std::string::npos);
        MINI_CHECK(irepr.find("my_instance_ref") != std::string::npos);
        MINI_CHECK(instcopy.guid() != inst.guid());
        MINI_CHECK(inst == instother);
        MINI_CHECK(inst != named);
        MINI_CHECK(named.name == "custom" && named.definition_guid == "def-9");
        MINI_CHECK(inst.features.empty());
        MINI_CHECK(inst != featured);
        MINI_CHECK(featuredcopy == featured && featuredcopy.guid() != featured.guid());
        MINI_CHECK(InstanceRef::FLAG_HIDDEN == 1 && InstanceRef::FLAG_LOCKED == 2 && InstanceRef::FLAG_COLOR == 4);
    }

    MINI_TEST("InstanceRef", "Transformation") {
        // using session_cpp::InstanceRef;
        // using session_cpp::Xform;

        InstanceRef inst("def", Xform::translation(1.0, 0.0, 0.0));
        const InstanceRef moved = inst.transformed(Xform::translation(5.0, 0.0, 0.0));
        inst.transform(Xform::translation(5.0, 0.0, 0.0));

        MINI_CHECK(TOLERANCE.is_close(moved[12], 6.0));
        MINI_CHECK(TOLERANCE.is_close(inst[12], 6.0));
    }

    MINI_TEST("InstanceRef", "Json Roundtrip") {
        // using session_cpp::ElementFeature;
        // using session_cpp::InstanceRef;
        // using session_cpp::Point;
        // using session_cpp::Polyline;
        // using session_cpp::Xform;

        InstanceRef inst("def-abc", Xform::translation(1.0, 2.0, 3.0));
        inst.name = "test_ref";
        inst.flags = 7;
        inst.features.push_back(ElementFeature("contact", 0, {Polyline(std::vector<Point>{Point(0, 0, 0), Point(1, 0, 0)})}));
        const std::string feature = inst.features[0].guid();

        const nlohmann::ordered_json j = inst.jsondump();
        const InstanceRef loaded_j = InstanceRef::jsonload(j);
        nlohmann::ordered_json bare = inst.jsondump();
        bare.erase("features");
        bare.erase("flags");
        const InstanceRef loaded_bare = InstanceRef::jsonload(bare);

        MINI_CHECK(loaded_j.name == "test_ref");
        MINI_CHECK(loaded_j.definition_guid == "def-abc");
        MINI_CHECK(loaded_j.flags == 7);
        MINI_CHECK(loaded_j.features.size() == 1);
        MINI_CHECK(loaded_j.features[0].guid() == feature);
        MINI_CHECK(loaded_j == inst);
        MINI_CHECK(loaded_bare.flags == 0 && loaded_bare.features.empty());
        MINI_CHECK(TOLERANCE.is_close(loaded_j[12], 1.0));

        const std::string s = inst.file_json_dumps();
        const InstanceRef loaded_s = InstanceRef::file_json_loads(s);

        MINI_CHECK(loaded_s.name == "test_ref");
        MINI_CHECK(loaded_s.definition_guid == "def-abc");

        const std::string filename = "serialization/test_instance_ref.json";
        inst.file_json_dump(filename);
        const InstanceRef loaded = InstanceRef::file_json_load(filename);

        MINI_CHECK(loaded.name == "test_ref");
        MINI_CHECK(loaded.definition_guid == "def-abc");
        MINI_CHECK(loaded.flags == 7);
        MINI_CHECK(TOLERANCE.is_close(loaded[12], 1.0));
        MINI_CHECK(TOLERANCE.is_close(loaded[13], 2.0));
        MINI_CHECK(TOLERANCE.is_close(loaded[14], 3.0));
    }

    MINI_TEST("InstanceRef", "Protobuf Roundtrip") {
        // using session_cpp::Color;
        // using session_cpp::ElementFeature;
        // using session_cpp::InstanceRef;
        // using session_cpp::Point;
        // using session_cpp::Polyline;
        // using session_cpp::Xform;

        const InstanceRef fresh;
        const session_proto::InstanceRef fresh_proto = fresh.to_proto();
        InstanceRef inst("def-xyz", Xform::translation(1.0, 2.0, 3.0));
        inst.name = "test_ref";
        inst.flags = 5;
        inst.features.push_back(ElementFeature("contact", 0, {Polyline(std::vector<Point>{Point(0, 0, 0), Point(1, 0, 0)})}));
        const std::string feature = inst.features[0].guid();
        InstanceRef plain("def-xyz", Xform::identity());
        plain.color = Color::red();
        const session_proto::InstanceRef plain_proto = plain.to_proto();
        const InstanceRef plain_loaded = InstanceRef::from_proto(plain_proto);

        const std::string guid = inst.guid();
        const std::string b = inst.pb_dumps();
        const InstanceRef loaded_b = InstanceRef::pb_loads(b);
        const InstanceRef converted = InstanceRef::from_proto(inst.to_proto());

        MINI_CHECK(!fresh.has_guid());
        MINI_CHECK(fresh_proto.guid().empty());
        MINI_CHECK(!plain_proto.has_xform());
        MINI_CHECK(!plain_proto.has_color());
        MINI_CHECK(plain_loaded.xform == Xform::identity());
        MINI_CHECK(plain_loaded.color == Color::white());
        MINI_CHECK(loaded_b.features.size() == 1);
        MINI_CHECK(loaded_b.features[0].guid() == feature);
        MINI_CHECK(loaded_b.name == "test_ref");
        MINI_CHECK(loaded_b.definition_guid == "def-xyz");
        MINI_CHECK(loaded_b.flags == 5);
        MINI_CHECK(loaded_b.guid() == guid);
        MINI_CHECK(TOLERANCE.is_close(loaded_b[14], 3.0));
        MINI_CHECK(converted == inst);
        MINI_CHECK(converted.guid() == guid);

        const std::string filename = "serialization/test_instance_ref.bin";
        inst.pb_dump(filename);
        const InstanceRef loaded = InstanceRef::pb_load(filename);

        MINI_CHECK(loaded.name == "test_ref");
        MINI_CHECK(loaded.definition_guid == "def-xyz");
        MINI_CHECK(loaded.guid() == guid);
        MINI_CHECK(TOLERANCE.is_close(loaded[12], 1.0));
        MINI_CHECK(TOLERANCE.is_close(loaded[13], 2.0));
        MINI_CHECK(TOLERANCE.is_close(loaded[14], 3.0));
    }

} // namespace session_cpp
