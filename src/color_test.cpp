#include "mini_test.h"
#include "color.h"
#include "color.pb.h"
#include "tolerance.h"
#include <array>
#include <exception>
#include <string>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Color", "Constructor") {
        // using session_cpp::Color;

        const Color cdefault;
        const Color clamped(-1.0f, 2.0f, 0.5f, 3.0f);
        Color c(1.0f, 0.0f, 0.0f, 1.0f, "red");
        const bool fresh = !c.has_guid();

        c[0] = 1.0f;
        c[1] = 0.0f;
        c[2] = 0.0f;
        c[3] = 1.0f;

        const float r = c[0];
        const float g = c[1];
        const float b = c[2];
        const float a = c[3];

        const std::string cstr = c.str();
        const std::string crepr = c.repr();

        const Color ccopy = c;
        const Color cother(1.0f, 0.0f, 0.0f, 1.0f, "red");

        MINI_CHECK(cdefault == Color(0.94f, 0.94f, 0.94f, 1.0f));
        MINI_CHECK(clamped == Color(0.0f, 1.0f, 0.5f, 1.0f));
        MINI_CHECK(fresh);
        MINI_CHECK(c.name == "red");
        MINI_CHECK(c.guid() != "");
        MINI_CHECK(c[0] == 1.0f && c[1] == 0.0f && c[2] == 0.0f && c[3] == 1.0f);
        MINI_CHECK(r == 1.0f && g == 0.0f && b == 0.0f && a == 1.0f);
        MINI_CHECK(cstr == "1.0, 0.0, 0.0, 1.0");
        MINI_CHECK(crepr == "Color(red, 1.0, 0.0, 0.0, 1.0)");
        MINI_CHECK(ccopy == cother);
        MINI_CHECK(c != Color::blue());
        MINI_CHECK(ccopy.guid() != c.guid());
    }

    MINI_TEST("Color", "Json Roundtrip") {
        // using session_cpp::Color;

        const Color c(1.0f, 0.5f, 0.25f, 1.0f, "test_color");

        const std::string guid = c.guid();
        const std::string filename = "serialization/test_color.json";
        c.file_json_dump(filename);

        const Color loaded = Color::file_json_load(filename);
        const Color parsed = Color::file_json_loads(c.file_json_dumps());

        MINI_CHECK(loaded.name == "test_color");
        MINI_CHECK(loaded[0] == 1.0f);
        MINI_CHECK(loaded[1] == 0.5f);
        MINI_CHECK(loaded[2] == 0.25f);
        MINI_CHECK(loaded[3] == 1.0f);
        MINI_CHECK(parsed == c);
        MINI_CHECK(loaded.guid() == guid);
        MINI_CHECK(parsed.guid() == guid);
    }

    MINI_TEST("Color", "Protobuf Roundtrip") {
        // using session_cpp::Color;

        const Color fresh;
        const session_proto::Color fresh_proto = fresh.to_proto();
        const Color c(1.0f, 0.5f, 0.25f, 1.0f, "test_color");

        const std::string guid = c.guid();
        const std::string filename = "serialization/test_color.bin";
        c.pb_dump(filename);

        const Color loaded = Color::pb_load(filename);
        const Color parsed = Color::pb_loads(c.pb_dumps());
        const Color converted = Color::from_proto(c.to_proto());

        MINI_CHECK(!fresh.has_guid());
        MINI_CHECK(fresh_proto.guid().empty());
        MINI_CHECK(loaded.name == "test_color");
        MINI_CHECK(loaded[0] == 1.0f);
        MINI_CHECK(loaded[1] == 0.5f);
        MINI_CHECK(loaded[2] == 0.25f);
        MINI_CHECK(loaded[3] == 1.0f);
        MINI_CHECK(parsed == c);
        MINI_CHECK(loaded.guid() == guid);
        MINI_CHECK(parsed.guid() == guid);
        MINI_CHECK(converted == c);
        MINI_CHECK(converted.guid() == guid);
    }

    MINI_TEST("Color", "Conversion") {
        // using session_cpp::Color;

        const Color c(1.0f, 0.5f, 0.25f, 1.0f);
        const std::array<float, 4> flts = c.to_unified_array();
        const Color back = Color::from_unified_array(flts);

        MINI_CHECK(TOLERANCE.is_close(flts[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(flts[1], 0.5));
        MINI_CHECK(TOLERANCE.is_close(flts[2], 0.25));
        MINI_CHECK(TOLERANCE.is_close(flts[3], 1.0));
        MINI_CHECK(back == c);
    }

    MINI_TEST("Color", "Presets") {
        // using session_cpp::Color;

        const Color white = Color::white();
        const Color black = Color::black();
        const Color grey = Color::grey();
        const Color red = Color::red();
        const Color orange = Color::orange();
        const Color yellow = Color::yellow();
        const Color lime = Color::lime();
        const Color green = Color::green();
        const Color mint = Color::mint();
        const Color cyan = Color::cyan();
        const Color azure = Color::azure();
        const Color blue = Color::blue();
        const Color violet = Color::violet();
        const Color magenta = Color::magenta();
        const Color pink = Color::pink();
        const Color maroon = Color::maroon();
        const Color brown = Color::brown();
        const Color olive = Color::olive();
        const Color teal = Color::teal();
        const Color navy = Color::navy();
        const Color purple = Color::purple();
        const Color silver = Color::silver();
        const Color lightgrey = Color::lightgrey();
        const std::vector<Color> palette = Color::palette();
        const std::vector<Color> expected = {
            red,
            orange,
            yellow,
            lime,
            green,
            mint,
            cyan,
            azure,
            blue,
            violet,
            magenta,
            pink
        };

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
        MINI_CHECK(lightgrey == Color(0.94f, 0.94f, 0.94f, 1.0f, "lightgrey"));
        MINI_CHECK(palette == expected);
    }

    MINI_TEST("Color", "Serialization Errors") {
        // using session_cpp::Color;

        const Color color;
        bool malformed_json = false;
        bool malformed_pb = false;
        bool json_write_failed = false;
        bool pb_write_failed = false;

        try {
            Color::file_json_loads("{}");
        } catch (const std::exception&) {
            malformed_json = true;
        }

        try {
            Color::pb_loads(std::string(1, static_cast<char>(0xff)));
        } catch (const std::runtime_error&) {
            malformed_pb = true;
        }

        try {
            color.file_json_dump("");
        } catch (const std::runtime_error&) {
            json_write_failed = true;
        }

        try {
            color.pb_dump("");
        } catch (const std::runtime_error&) {
            pb_write_failed = true;
        }

        MINI_CHECK(malformed_json);
        MINI_CHECK(malformed_pb);
        MINI_CHECK(json_write_failed);
        MINI_CHECK(pb_write_failed);
    }

} // namespace session_cpp
