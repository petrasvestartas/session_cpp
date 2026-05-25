#include "mini_test.h"
#include "color.h"
#include "tolerance.h"

#include <cmath>
#include <filesystem>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Color", "Constructor"){
        // uncomment #include "color.h"

        // Constructor
        Color c(1.0f, 0.0f, 0.0f, 1.0f, "red");

        // Setters
        c[0] = 1.0f;
        c[1] = 0.0f;
        c[2] = 0.0f;
        c[3] = 1.0f;

        // Getters
        float r = c[0];
        float g = c[1];
        float b = c[2];
        float a = c[3];

        // Minimal and Full String Representation
        std::string cstr = c.str();
        std::string crepr = c.repr();

        // Copy (duplicates everything except guid())
        Color ccopy = c;
        Color cother(1.0f, 0.0f, 0.0f, 1.0f, "red");

        MINI_CHECK(c.name == "red");
        MINI_CHECK(c.guid() != "");
        MINI_CHECK(c[0] == 1.0f && c[1] == 0.0f && c[2] == 0.0f && c[3] == 1.0f);
        MINI_CHECK(r == 1.0f && g == 0.0f && b == 0.0f && a == 1.0f);
        MINI_CHECK(cstr == "1.0, 0.0, 0.0, 1.0");
        MINI_CHECK(crepr == "Color(red, 1.0, 0.0, 0.0, 1.0)");
        MINI_CHECK(ccopy == cother);
        MINI_CHECK(ccopy.guid() != c.guid());
    }

    MINI_TEST("Color", "Json Roundtrip"){
      // uncomment #include "color.h"

      Color c(1.0f, 0.5f, 0.25f, 1.0f, "serialization/test_color");

      //   jsondump()      │ ordered_json │ to JSON object (internal use)
      //   jsonload(j)     │ ordered_json │ from JSON object (internal use)
      //   file_json_dumps()    │ std::string  │ to JSON string
      //   file_json_loads(s)   │ std::string  │ from JSON string
      //   file_json_dump(path) │ file         │ write to file
      //   file_json_load(path) │ file         │ read from file

      std::string filename = "serialization/test_color.json";
      c.file_json_dump(filename);
      Color loaded = Color::file_json_load(filename);

      MINI_CHECK(loaded.name == "serialization/test_color");
      MINI_CHECK(loaded[0] == 1.0f);
      MINI_CHECK(loaded[1] == 0.5f);
      MINI_CHECK(loaded[2] == 0.25f);
      MINI_CHECK(loaded[3] == 1.0f);
    }

    MINI_TEST("Color", "Protobuf Roundtrip"){
      // uncomment #include "color.h"

      Color c(1.0f, 0.5f, 0.25f, 1.0f, "serialization/test_color");

      std::string filename = "serialization/test_color.bin";
      c.pb_dump(filename);
      Color loaded = Color::pb_load(filename);

      MINI_CHECK(loaded.name == "serialization/test_color");
      MINI_CHECK(loaded[0] == 1.0f);
      MINI_CHECK(loaded[1] == 0.5f);
      MINI_CHECK(loaded[2] == 0.25f);
      MINI_CHECK(loaded[3] == 1.0f);
    }

    MINI_TEST("Color", "Conversion"){
      // uncomment #include "color.h"

      Color c(1.0f, 0.5f, 0.25f, 1.0f);
      std::array<float, 4> flts = c.to_unified_array();
      Color ints = Color::from_unified_array(flts);

      MINI_CHECK(TOLERANCE.is_close(flts[0], 1.0));
      MINI_CHECK(TOLERANCE.is_close(flts[1], 0.5));
      MINI_CHECK(TOLERANCE.is_close(flts[2], 0.25));
      MINI_CHECK(TOLERANCE.is_close(flts[3], 1.0));
      MINI_CHECK(ints == c);
    }

    MINI_TEST("Color", "Presets"){
      // uncomment #include "color.h"

      Color white = Color::white();
      Color black = Color::black();
      Color grey = Color::grey();
      Color red = Color::red();
      Color orange = Color::orange();
      Color yellow = Color::yellow();
      Color lime = Color::lime();
      Color green = Color::green();
      Color mint = Color::mint();
      Color cyan = Color::cyan();
      Color azure = Color::azure();
      Color blue = Color::blue();
      Color violet = Color::violet();
      Color magenta = Color::magenta();
      Color pink = Color::pink();
      Color maroon = Color::maroon();
      Color brown = Color::brown();
      Color olive = Color::olive();
      Color teal = Color::teal();
      Color navy = Color::navy();
      Color purple = Color::purple();
      Color silver = Color::silver();

      MINI_CHECK(white == Color(1.0f, 1.0f, 1.0f, 1.0f, "white"));
      MINI_CHECK(black == Color(0.0f, 0.0f, 0.0f, 1.0f, "black"));
      MINI_CHECK(grey == Color(0.5f, 0.5f, 0.5f, 1.0f, "grey"));
      MINI_CHECK(red == Color(1.0f, 0.0f, 0.0f, 1.0f, "red"));
      MINI_CHECK(orange == Color(1.0f, 0.5f, 0.0f, 1.0f, "orange"));
      MINI_CHECK(yellow == Color(1.0f, 1.0f, 0.0f, 1.0f, "yellow"));
      MINI_CHECK(lime == Color(0.5f, 1.0f, 0.0f, 1.0f, "lime"));
      MINI_CHECK(green == Color(0.0f, 1.0f, 0.0f, 1.0f, "green"));
      MINI_CHECK(mint == Color(0.0f, 1.0f, 0.5f, 1.0f, "mint"));
      MINI_CHECK(cyan == Color(0.0f, 1.0f, 1.0f, 1.0f, "cyan"));
      MINI_CHECK(azure == Color(0.0f, 0.5f, 1.0f, 1.0f, "azure"));
      MINI_CHECK(blue == Color(0.0f, 0.0f, 1.0f, 1.0f, "blue"));
      MINI_CHECK(violet == Color(0.5f, 0.0f, 1.0f, 1.0f, "violet"));
      MINI_CHECK(magenta == Color(1.0f, 0.0f, 1.0f, 1.0f, "magenta"));
      MINI_CHECK(pink == Color(1.0f, 0.0f, 0.5f, 1.0f, "pink"));
      MINI_CHECK(maroon == Color(0.5f, 0.0f, 0.0f, 1.0f, "maroon"));
      MINI_CHECK(brown == Color(0.5f, 0.25f, 0.0f, 1.0f, "brown"));
      MINI_CHECK(olive == Color(0.5f, 0.5f, 0.0f, 1.0f, "olive"));
      MINI_CHECK(teal == Color(0.0f, 0.5f, 0.5f, 1.0f, "teal"));
      MINI_CHECK(navy == Color(0.0f, 0.0f, 0.5f, 1.0f, "navy"));
      MINI_CHECK(purple == Color(0.5f, 0.0f, 0.5f, 1.0f, "purple"));
      MINI_CHECK(silver == Color(0.75f, 0.75f, 0.75f, 1.0f, "silver"));
    }

}
