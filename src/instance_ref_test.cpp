#include "mini_test.h"
#include "instance_ref.h"
#include "xform.h"
#include "color.h"
#include "tolerance.h"

#include <string>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("InstanceRef", "Constructor") {
    // uncomment #include "instance_ref.h"
    // uncomment #include "xform.h"
    // uncomment #include "color.h"

    // Constructor from a definition guid and a placement transform
    Xform x = Xform::translation(10.0, 20.0, 30.0);
    InstanceRef ref("def-123", x);

    // Setter on a copy (keep ref pristine for the == check below)
    InstanceRef refset = ref;
    refset[0] = 2.0;
    double m0 = refset[0];

    // Minimal and Full String Representation
    std::string rstr = ref.str();
    std::string rrepr = ref.repr();

    // Copy (duplicate everything but guid())
    InstanceRef rcopy = ref;
    InstanceRef rother("def-123", x);

    // with_name constructor
    InstanceRef rwn = InstanceRef::with_name("custom", "def-9", Xform::identity());

    MINI_CHECK(ref.name == "my_instance_ref");
    MINI_CHECK(ref.definition_guid == "def-123");
    MINI_CHECK(!ref.guid().empty());
    MINI_CHECK(m0 == 2.0);
    MINI_CHECK(ref[12] == 10.0 && ref[13] == 20.0 && ref[14] == 30.0);
    MINI_CHECK(rstr.find("def-123") != std::string::npos);
    MINI_CHECK(rrepr.find("InstanceRef") != std::string::npos);
    MINI_CHECK(rrepr.find("my_instance_ref") != std::string::npos);
    MINI_CHECK(rcopy.guid() != ref.guid());
    MINI_CHECK(ref == rother);
    MINI_CHECK(ref != rwn);
    MINI_CHECK(rwn.name == "custom" && rwn.definition_guid == "def-9");
}

MINI_TEST("InstanceRef", "Transformation") {
    // uncomment #include "instance_ref.h"
    // uncomment #include "xform.h"

    InstanceRef ref("def", Xform::translation(1.0, 0.0, 0.0));
    InstanceRef moved = ref.transformed(Xform::translation(5.0, 0.0, 0.0)); // Make a copy
    ref.transform(Xform::translation(5.0, 0.0, 0.0)); // compose in place

    // translation(5) * translation(1) => translation(6)
    MINI_CHECK(TOLERANCE.is_close(moved[12], 6.0));
    MINI_CHECK(TOLERANCE.is_close(ref[12], 6.0));
}

MINI_TEST("InstanceRef", "Json Roundtrip") {
    // uncomment #include "instance_ref.h"

    InstanceRef ref("def-abc", Xform::translation(1.0, 2.0, 3.0));
    ref.name = "test_ref";
    ref.flags = 7;

    // JSON object
    nlohmann::ordered_json j = ref.jsondump();
    InstanceRef loaded_j = InstanceRef::jsonload(j);

    MINI_CHECK(loaded_j.name == "test_ref");
    MINI_CHECK(loaded_j.definition_guid == "def-abc");
    MINI_CHECK(loaded_j.flags == 7);
    MINI_CHECK(TOLERANCE.is_close(loaded_j[12], 1.0));

    // String
    std::string s = ref.file_json_dumps();
    InstanceRef loaded_s = InstanceRef::file_json_loads(s);
    MINI_CHECK(loaded_s.name == "test_ref");
    MINI_CHECK(loaded_s.definition_guid == "def-abc");

    // File
    std::string fname = "serialization/test_instance_ref.json";
    ref.file_json_dump(fname);
    InstanceRef loaded = InstanceRef::file_json_load(fname);
    MINI_CHECK(loaded.name == "test_ref");
    MINI_CHECK(loaded.definition_guid == "def-abc");
    MINI_CHECK(loaded.flags == 7);
    MINI_CHECK(TOLERANCE.is_close(loaded[12], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded[13], 2.0));
    MINI_CHECK(TOLERANCE.is_close(loaded[14], 3.0));
}

MINI_TEST("InstanceRef", "Protobuf Roundtrip") {
    // uncomment #include "instance_ref.h"

    InstanceRef ref("def-xyz", Xform::translation(1.0, 2.0, 3.0));
    ref.name = "test_ref";
    ref.flags = 5;

    // String
    std::string s = ref.pb_dumps();
    InstanceRef loaded_s = InstanceRef::pb_loads(s);

    MINI_CHECK(loaded_s.name == "test_ref");
    MINI_CHECK(loaded_s.definition_guid == "def-xyz");
    MINI_CHECK(loaded_s.flags == 5);
    MINI_CHECK(loaded_s.guid() == ref.guid());
    MINI_CHECK(TOLERANCE.is_close(loaded_s[14], 3.0));

    // File
    std::string fname = "serialization/test_instance_ref.bin";
    ref.pb_dump(fname);
    InstanceRef loaded = InstanceRef::pb_load(fname);
    MINI_CHECK(loaded.name == "test_ref");
    MINI_CHECK(loaded.definition_guid == "def-xyz");
    MINI_CHECK(loaded.guid() == ref.guid());
    MINI_CHECK(TOLERANCE.is_close(loaded[12], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded[13], 2.0));
    MINI_CHECK(TOLERANCE.is_close(loaded[14], 3.0));
}

} // namespace session_cpp
