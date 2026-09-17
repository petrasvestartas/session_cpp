#include "mini_test.h"
#include "instance_ref.h"
#include "xform.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("InstanceRef", "Constructor") {

  Xform x = Xform::translation(10.0, 20.0, 30.0);
  InstanceRef inst("def-123", x);

  InstanceRef instset = inst;
  instset[0] = 2.0;
  double m0 = instset[0];

  std::string istr = inst.str();
  std::string irepr = inst.repr();

  InstanceRef instcopy = inst;
  InstanceRef instother("def-123", x);
  InstanceRef named = InstanceRef::with_name("custom", "def-9", Xform::identity());

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
}

MINI_TEST("InstanceRef", "Transformation") {

  InstanceRef inst("def", Xform::translation(1.0, 0.0, 0.0));
  InstanceRef moved = inst.transformed(Xform::translation(5.0, 0.0, 0.0));
  inst.transform(Xform::translation(5.0, 0.0, 0.0));

  MINI_CHECK(TOLERANCE.is_close(moved[12], 6.0));
  MINI_CHECK(TOLERANCE.is_close(inst[12], 6.0));
}

MINI_TEST("InstanceRef", "Json Roundtrip") {

  InstanceRef inst("def-abc", Xform::translation(1.0, 2.0, 3.0));
  inst.name = "test_ref";
  inst.flags = 7;

  nlohmann::ordered_json j = inst.jsondump();
  InstanceRef loaded_j = InstanceRef::jsonload(j);

  MINI_CHECK(loaded_j.name == "test_ref");
  MINI_CHECK(loaded_j.definition_guid == "def-abc");
  MINI_CHECK(loaded_j.flags == 7);
  MINI_CHECK(TOLERANCE.is_close(loaded_j[12], 1.0));

  std::string s = inst.file_json_dumps();
  InstanceRef loaded_s = InstanceRef::file_json_loads(s);

  MINI_CHECK(loaded_s.name == "test_ref");
  MINI_CHECK(loaded_s.definition_guid == "def-abc");

  std::string filename = "serialization/test_instance_ref.json";
  inst.file_json_dump(filename);
  InstanceRef loaded = InstanceRef::file_json_load(filename);

  MINI_CHECK(loaded.name == "test_ref");
  MINI_CHECK(loaded.definition_guid == "def-abc");
  MINI_CHECK(loaded.flags == 7);
  MINI_CHECK(TOLERANCE.is_close(loaded[12], 1.0));
  MINI_CHECK(TOLERANCE.is_close(loaded[13], 2.0));
  MINI_CHECK(TOLERANCE.is_close(loaded[14], 3.0));
}

MINI_TEST("InstanceRef", "Protobuf Roundtrip") {

  InstanceRef inst("def-xyz", Xform::translation(1.0, 2.0, 3.0));
  inst.name = "test_ref";
  inst.flags = 5;

  std::string guid = inst.guid();
  std::string b = inst.pb_dumps();
  InstanceRef loaded_b = InstanceRef::pb_loads(b);

  MINI_CHECK(loaded_b.name == "test_ref");
  MINI_CHECK(loaded_b.definition_guid == "def-xyz");
  MINI_CHECK(loaded_b.flags == 5);
  MINI_CHECK(loaded_b.guid() == guid);
  MINI_CHECK(TOLERANCE.is_close(loaded_b[14], 3.0));

  std::string filename = "serialization/test_instance_ref.bin";
  inst.pb_dump(filename);
  InstanceRef loaded = InstanceRef::pb_load(filename);

  MINI_CHECK(loaded.name == "test_ref");
  MINI_CHECK(loaded.definition_guid == "def-xyz");
  MINI_CHECK(loaded.guid() == guid);
  MINI_CHECK(TOLERANCE.is_close(loaded[12], 1.0));
  MINI_CHECK(TOLERANCE.is_close(loaded[13], 2.0));
  MINI_CHECK(TOLERANCE.is_close(loaded[14], 3.0));
}

} // namespace session_cpp
