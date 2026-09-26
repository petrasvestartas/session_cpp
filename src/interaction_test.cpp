#include "mini_test.h"
#include "interaction.h"
#include "interaction.pb.h"
#include <memory>
#include <string>
#include <type_traits>

using namespace session_cpp::mini_test;

namespace session_cpp {

namespace {

/// A test-only subclass: a named interaction with no state of its own.
class NamedInteraction : public Interaction {
public:
    /// Construct from a name.
    NamedInteraction(const std::string& name = "")
        : Interaction(name) {}

    /// Return the registered type name.
    std::string interaction_type_name() const override {
        return "NamedInteraction";
    }

    /// Return no state.
    std::string interaction_data_dumps() const override {
        return "";
    }

    /// Return a copy with the same guid.
    std::shared_ptr<Interaction> clone() const override {
        return std::make_shared<NamedInteraction>(*this);
    }
};

/// Build a NamedInteraction from its data.
std::shared_ptr<Interaction> named_interaction(const std::string&) {
    return std::make_shared<NamedInteraction>();
}

} // namespace

MINI_TEST("Interaction", "Constructor") {
    // using session_cpp::Interaction;

    const NamedInteraction unnamed;
    const NamedInteraction glue("glue");
    const NamedInteraction duplicate = glue;
    const std::shared_ptr<Interaction> cloned = glue.clone();

    MINI_CHECK(unnamed.name.empty());
    MINI_CHECK(glue.name == "glue");
    MINI_CHECK(glue.interaction_type_name() == "NamedInteraction");
    MINI_CHECK(duplicate == glue);
    MINI_CHECK(duplicate.guid() == glue.guid());
    MINI_CHECK(*cloned == glue);
    MINI_CHECK(cloned->guid() == glue.guid());
    MINI_CHECK(unnamed != glue);
    MINI_CHECK(unnamed.guid() != glue.guid());
    MINI_CHECK(glue.str() == "NamedInteraction(glue)");
    MINI_CHECK(glue.repr() == "NamedInteraction(" + glue.guid() + ", glue)");
}

MINI_TEST("Interaction", "Abstract Base") {
    // using session_cpp::Interaction;

    const std::shared_ptr<Interaction> glue = std::make_shared<NamedInteraction>("glue");

    MINI_CHECK(std::is_abstract_v<Interaction>);
    MINI_CHECK(glue->interaction_type_name() == "NamedInteraction");
}

MINI_TEST("Interaction", "Json Roundtrip") {
    // using session_cpp::Interaction;

    Interaction::register_type("NamedInteraction", named_interaction);
    const NamedInteraction glue("glue");

    const nlohmann::ordered_json data = glue.jsondump();
    const std::shared_ptr<Interaction> loaded_j = Interaction::jsonload(data);
    const std::shared_ptr<Interaction> loaded_s = Interaction::file_json_loads(glue.file_json_dumps());

    const std::string filename = "serialization/test_interaction.json";
    glue.file_json_dump(filename);
    const std::shared_ptr<Interaction> loaded = Interaction::file_json_load(filename);

    MINI_CHECK(data["type"] == "Interaction");
    MINI_CHECK(*loaded_j == glue);
    MINI_CHECK(*loaded_s == glue);
    MINI_CHECK(*loaded == glue);
    MINI_CHECK(loaded->guid() == glue.guid());
}

MINI_TEST("Interaction", "Protobuf Roundtrip") {
    // using session_cpp::Interaction;

    Interaction::register_type("NamedInteraction", named_interaction);
    const NamedInteraction glue("glue");

    const session_proto::Interaction proto = glue.to_proto();
    const std::shared_ptr<Interaction> converted = Interaction::from_proto(proto);
    const std::shared_ptr<Interaction> loaded_b = Interaction::pb_loads(glue.pb_dumps());

    const std::string filename = "serialization/test_interaction.bin";
    glue.pb_dump(filename);
    const std::shared_ptr<Interaction> loaded = Interaction::pb_load(filename);

    MINI_CHECK(proto.interaction_type() == "NamedInteraction");
    MINI_CHECK(*converted == glue);
    MINI_CHECK(*loaded_b == glue);
    MINI_CHECK(*loaded == glue);
    MINI_CHECK(loaded->guid() == glue.guid());
}

MINI_TEST("Interaction", "Registry Unknown Type") {
    // using session_cpp::Interaction;
    // using session_cpp::InteractionUnknown;

    session_proto::Interaction proto = NamedInteraction("mystery").to_proto();
    proto.set_interaction_type("NeverRegistered");
    proto.set_interaction_data("whatever this package meant");
    const std::shared_ptr<Interaction> loaded = Interaction::pb_loads(proto.SerializeAsString());
    const std::shared_ptr<Interaction> saved = Interaction::pb_loads(loaded->pb_dumps());

    MINI_CHECK(!Interaction::is_registered("NeverRegistered"));
    MINI_CHECK(dynamic_cast<const InteractionUnknown*>(loaded.get()) != nullptr);
    MINI_CHECK(loaded->name == "mystery");
    MINI_CHECK(loaded->interaction_type_name() == "NeverRegistered");
    MINI_CHECK(loaded->interaction_data_dumps() == "whatever this package meant");
    MINI_CHECK(*saved == *loaded);
}

} // namespace session_cpp
