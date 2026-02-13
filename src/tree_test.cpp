#include "mini_test.h"
#include "tree.h"
#include "encoders.h"
#include "point.h"
#include "tolerance.h"
#include <filesystem>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Tree", "Json_roundtrip") {
    Tree original("./serialization/test_tree");
    auto point1 = std::make_shared<Point>(1.0, 2.0, 3.0);
    auto node1 = std::make_shared<TreeNode>(point1->guid);
    original.add(node1, nullptr);

    std::filesystem::create_directories("./serialization");
    encoders::json_dump(original, "./serialization/test_tree.json");
    Tree loaded = encoders::json_load<Tree>("./serialization/test_tree.json");

    MINI_CHECK(loaded.name == original.name);
    MINI_CHECK(loaded.nodes().size() == original.nodes().size());
}

} // namespace session_cpp
