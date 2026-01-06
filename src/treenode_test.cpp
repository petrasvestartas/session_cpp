#include "catch_amalgamated.hpp"
#include "treenode.h"
#include "encoders.h"
#include <filesystem>

using namespace session_cpp;

TEST_CASE("TreeNode JSON roundtrip", "[treenode]") {
    auto original = std::make_shared<TreeNode>("test_node");
    auto child = std::make_shared<TreeNode>("child_node");
    original->add(child);

    std::filesystem::create_directories("../serialization");
    encoders::json_dump(original->jsondump(), "../serialization/test_treenode.json");
    auto loaded_json = encoders::json_load_data("../serialization/test_treenode.json");
    auto loaded = TreeNode::jsonload(loaded_json);
    
    REQUIRE(loaded->name == original->name);
    
    std::filesystem::remove("../serialization/test_treenode.json");
}
