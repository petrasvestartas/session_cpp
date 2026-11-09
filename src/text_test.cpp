#include "mini_test.h"
#include "objects.h"
#include "session.h"
#include "text.h"
#include <cstdio>
#include <string>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("Text", "Constructor") {

    Text text("n_0", Point(1.0, 2.0, 3.0));
    text.name = "n0";
    text.math = true;
    text.spatial = true;
    text.outline = -1.5;
    const Text copy = text.duplicate();
    const Text other;
    const Text spatial = Text::from_tex("n_0", Point(1.0, 2.0, 3.0), true);
    const Text flat = Text::from_tex("x_0", Point(0.0, 0.0, 0.0));
    const std::string str = text.str();
    const std::string repr = text.repr();

    MINI_CHECK(text.name == "n0" && !text.guid().empty());
    MINI_CHECK(text.at == Point(1.0, 2.0, 3.0));
    MINI_CHECK(text.height == 28.0 && text.color == Color::white());
    MINI_CHECK(text.math && text.spatial && !text.serif);
    MINI_CHECK(copy == text && copy.guid() != text.guid());
    MINI_CHECK(other != text);
    MINI_CHECK(spatial.math && spatial.height == 18.0 && spatial.outline == -1.5 && !spatial.serif);
    MINI_CHECK(flat.math && !flat.spatial && flat.height == 28.0 && flat.outline == 0.0 && flat.serif);
    MINI_CHECK(str == "n_0");
    MINI_CHECK(repr.find("Text(") != std::string::npos);
}

MINI_TEST("Text", "Json") {

    Text text("q=\\frac{f}{l}", Point(4.0, 5.0, 6.0));
    text.math = true;
    text.serif = true;
    const std::string filename = "text_test.json";
    const Text loaded = Text::file_json_loads(text.file_json_dumps());
    text.file_json_dump(filename);
    const Text from_file = Text::file_json_load(filename);
    std::remove(filename.c_str());

    MINI_CHECK(loaded == text);
    MINI_CHECK(from_file == text);
}

MINI_TEST("Text", "Protobuf") {

    Text text("x_0", Point(7.0, 8.0, 9.0));
    text.math = true;
    text.spatial = true;
    text.outline = -1.5;
    const std::string filename = "text_test.pb";
    const Text loaded = Text::pb_loads(text.pb_dumps());
    text.pb_dump(filename);
    const Text from_file = Text::pb_load(filename);
    std::remove(filename.c_str());

    MINI_CHECK(loaded == text);
    MINI_CHECK(from_file == text);
}

MINI_TEST("Text", "Component") {

    Text text("C[0,1]", Point(10.0, 11.0, 0.0));
    text.math = true;
    const Component component = text.to_component();
    const Text loaded = Text::from_component(component);
    Session session("text");
    session.add_text(text);
    const Session restored = Session::pb_loads(session.pb_dumps());

    MINI_CHECK(component.type_name == "Text");
    MINI_CHECK(component.guid() == text.guid());
    MINI_CHECK(loaded == text);
    MINI_CHECK(session.objects.components->size() == 1);
    MINI_CHECK((*session.objects.components)[0]->type_name == "Text");
    MINI_CHECK(Text::from_component(*restored.objects.components->at(0)) == text);
}

} // namespace session_cpp
