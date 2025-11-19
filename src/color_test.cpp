 #include "mini_test.h"
 #include "color.h"
 #include "encoders.h"
 #include <filesystem>

 using namespace session_cpp;
 using namespace session_cpp::mini_test;

 MINI_TEST(Color, json_roundtrip) {
   Color original(100, 150, 200, 255);
   original.name = "test_color";

   encoders::json_dump(original, "test_color.json");
   Color loaded = encoders::json_load<Color>("test_color.json");

   MINI_CHECK(loaded.r == original.r);
   MINI_CHECK(loaded.g == original.g);
   MINI_CHECK(loaded.b == original.b);
   MINI_CHECK(loaded.a == original.a);
   MINI_CHECK(loaded.name == original.name);

   std::filesystem::remove("test_color.json");
 }
