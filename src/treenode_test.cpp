#include "mini_test.h"
#include "treenode.h"
#include "encoders.h"
#include "tolerance.h"
#include <filesystem>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("TreeNode", "Json_roundtrip") {
    auto original = std::make_shared<TreeNode>("test_node");
    auto child = std::make_shared<TreeNode>("child_node");
    original->add(child);

    std::filesystem::create_directories("./serialization");
    encoders::json_dump(original->jsondump(), "./serialization/test_treenode.json");
    auto loaded_json = encoders::json_load_data("./serialization/test_treenode.json");
    auto loaded = TreeNode::jsonload(loaded_json);

    MINI_CHECK(loaded->name == original->name);

    std::filesystem::remove("./serialization/test_treenode.json");
}

} // namespace session_cpp
