#include "mini_test.h"
#include "color.h"
#include "tolerance.h"

#include <cmath>
#include <filesystem>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Color", "constructor"){
        // uncomment #include "color.h"

        // Constructor
        Color c(255, 0, 0, 255, "red");

        // Setters
        c[0] = 255;
        c[1] = 0;
        c[2] = 0;
        c[3] = 255;

        // Getters
        int r = c[0];
        int g = c[1];
        int b = c[2];
        int a = c[3];

        // Minimal and Full String Representation
        std::string cstr = c.str();
        std::string crepr = c.repr();

        // Copy (duplicates everything except guid)
        Color ccopy = c;
        Color cother(255, 0, 0, 255, "red");

        MINI_CHECK(c.name == "red" && c.guid != "" && c[0] == 255 && c[1] == 0 && c[2] == 0 && c[3] == 255);
        MINI_CHECK(r == 255 && g == 0 && b == 0 && a == 255);
        MINI_CHECK(cstr == "255, 0, 0, 255");
        MINI_CHECK(crepr == "Color(red, 255, 0, 0, 255)");
        MINI_CHECK(ccopy == cother);
        MINI_CHECK(ccopy.guid != c.guid);
    }

    MINI_TEST("Color", "json_roundtrip"){
      // uncomment #include "color.h"

      Color c(255, 128, 64, 255, "serialization/test_color");

      std::string filename = "serialization/test_color.json";
      c.json_dump(filename);
      Color loaded = Color::json_load(filename);

      MINI_CHECK(loaded.name == "serialization/test_color");
      MINI_CHECK(loaded[0] == 255);
      MINI_CHECK(loaded[1] == 128);
      MINI_CHECK(loaded[2] == 64);
      MINI_CHECK(loaded[3] == 255);
    }

#ifdef ENABLE_PROTOBUF
    MINI_TEST("Color", "protobuf_roundtrip"){
      // uncomment #include "color.h"

      Color c(255, 128, 64, 255, "serialization/test_color");

      std::string filename = "serialization/test_color.bin";
      c.protobuf_dump(filename);
      Color loaded = Color::protobuf_load(filename);

      MINI_CHECK(loaded.name == "serialization/test_color");
      MINI_CHECK(loaded[0] == 255);
      MINI_CHECK(loaded[1] == 128);
      MINI_CHECK(loaded[2] == 64);
      MINI_CHECK(loaded[3] == 255);
    }
#endif

    MINI_TEST("Color", "conversion"){
      // uncomment #include "color.h"

      Color c(255, 128, 64, 255);
      std::array<double, 4> flts = c.to_unified_array();
      Color ints = Color::from_unified_array(flts);

      MINI_CHECK(TOLERANCE.is_close(flts[0], 1.0));
      MINI_CHECK(TOLERANCE.is_close(flts[1], 0.50196078));
      MINI_CHECK(TOLERANCE.is_close(flts[2], 0.25098039));
      MINI_CHECK(TOLERANCE.is_close(flts[3], 1.0));
      MINI_CHECK(ints == c);
    }

    MINI_TEST("Color", "presets"){
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
      MINI_CHECK(white == Color(255, 255, 255, 255, "white"));
      MINI_CHECK(black == Color(0, 0, 0, 255, "black"));
      MINI_CHECK(grey == Color(128, 128, 128, 255, "grey"));
      MINI_CHECK(red == Color(255, 0, 0, 255, "red"));
      MINI_CHECK(orange == Color(255, 128, 0, 255, "orange"));
      MINI_CHECK(yellow == Color(255, 255, 0, 255, "yellow"));
      MINI_CHECK(lime == Color(128, 255, 0, 255, "lime"));
      MINI_CHECK(green == Color(0, 255, 0, 255, "green"));
      MINI_CHECK(mint == Color(0, 255, 128, 255, "mint"));
      MINI_CHECK(cyan == Color(0, 255, 255, 255, "cyan"));
      MINI_CHECK(azure == Color(0, 128, 255, 255, "azure"));
      MINI_CHECK(blue == Color(0, 0, 255, 255, "blue"));
      MINI_CHECK(violet == Color(128, 0, 255, 255, "violet"));
      MINI_CHECK(magenta == Color(255, 0, 255, 255, "magenta"));
      MINI_CHECK(pink == Color(255, 0, 128, 255, "pink"));
      MINI_CHECK(maroon == Color(128, 0, 0, 255, "maroon"));
      MINI_CHECK(brown == Color(128, 64, 0, 255, "brown"));
      MINI_CHECK(olive == Color(128, 128, 0, 255, "olive"));
      MINI_CHECK(teal == Color(0, 128, 128, 255, "teal"));
      MINI_CHECK(navy == Color(0, 0, 128, 255, "navy"));
      MINI_CHECK(purple == Color(128, 0, 128, 255, "purple"));
      MINI_CHECK(silver == Color(192, 192, 192, 255, "silver"));
    }

}
