#include "mini_test.h"
#include "tree.h"
#include "treenode.h"
#include "encoders.h"

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Tree", "Constructor") {
    // uncomment #include "tree.h"

    // Default constructor
    Tree t0;

    // Constructor with name
    Tree t("my_named_tree");

    // Minimal string representation
    std::string tstr = t.str();

    MINI_CHECK(t0.name == "my_tree");
    MINI_CHECK(!t0.guid().empty());
    MINI_CHECK(t.name == "my_named_tree");
    MINI_CHECK(tstr.find("Tree") != std::string::npos);
}

MINI_TEST("Tree", "Json Roundtrip") {
    // uncomment #include "tree.h"
    // uncomment #include "treenode.h"

    Tree original("test_tree");
    auto root_node = std::make_shared<TreeNode>("root_node");
    original.add(root_node);

    //   jsondump()      │ ordered_json │ to JSON object (internal use)
    //   jsonload(j)     │ ordered_json │ from JSON object (internal use)
    //   json_dumps()    │ std::string  │ to JSON string
    //   json_loads(s)   │ std::string  │ from JSON string
    //   json_dump(path) │ file         │ write to file
    //   json_load(path) │ file         │ read from file

    std::string fname = "serialization/test_tree.json";
    original.json_dump(fname);
    Tree loaded = Tree::json_load(fname);

    MINI_CHECK(loaded.name == original.name);
    MINI_CHECK(loaded.nodes().size() == original.nodes().size());
}

MINI_TEST("Tree", "Protobuf Roundtrip") {
    // uncomment #include "tree.h"
    // uncomment #include "treenode.h"

    Tree original("test_tree");
    auto root_node = std::make_shared<TreeNode>("root_node");
    original.add(root_node);

    std::string fname = "serialization/test_tree.bin";
    original.pb_dump(fname);
    Tree loaded = Tree::pb_load(fname);

    MINI_CHECK(loaded.name == original.name);
    MINI_CHECK(loaded.nodes().size() == original.nodes().size());
}

MINI_TEST("Tree", "Root") {
    // uncomment #include "tree.h"
    // uncomment #include "treenode.h"

    Tree t("t");
    auto root = std::make_shared<TreeNode>("root");
    t.add(root);

    MINI_CHECK(t.root() == root);
}

MINI_TEST("Tree", "Add") {
    // uncomment #include "tree.h"
    // uncomment #include "treenode.h"

    Tree t("t");
    auto root = std::make_shared<TreeNode>("root");
    auto child = std::make_shared<TreeNode>("child");
    t.add(root);
    t.add(child, root);

    MINI_CHECK(t.nodes().size() == 2);
}

MINI_TEST("Tree", "Nodes") {
    // uncomment #include "tree.h"
    // uncomment #include "treenode.h"

    Tree t("t");
    auto root = std::make_shared<TreeNode>("root");
    auto child = std::make_shared<TreeNode>("child");
    t.add(root);
    t.add(child, root);

    auto all = t.nodes();

    MINI_CHECK(all.size() == 2);
    MINI_CHECK(all[0]->name == "root");
    MINI_CHECK(all[1]->name == "child");
}

MINI_TEST("Tree", "Remove") {
    // uncomment #include "tree.h"
    // uncomment #include "treenode.h"

    Tree t("t");
    auto root = std::make_shared<TreeNode>("root");
    auto child = std::make_shared<TreeNode>("child");
    t.add(root);
    t.add(child, root);
    t.remove(child);

    MINI_CHECK(t.nodes().size() == 1);
}

MINI_TEST("Tree", "Leaves") {
    // uncomment #include "tree.h"
    // uncomment #include "treenode.h"

    Tree t("t");
    auto root = std::make_shared<TreeNode>("root");
    auto a = std::make_shared<TreeNode>("a");
    auto b = std::make_shared<TreeNode>("b");
    t.add(root);
    t.add(a, root);
    t.add(b, root);

    auto lvs = t.leaves();

    MINI_CHECK(lvs.size() == 2);
    MINI_CHECK(lvs[0]->name == "a");
    MINI_CHECK(lvs[1]->name == "b");
}

MINI_TEST("Tree", "Traverse") {
    // uncomment #include "tree.h"
    // uncomment #include "treenode.h"

    Tree t("t");
    auto root = std::make_shared<TreeNode>("root");
    auto a = std::make_shared<TreeNode>("a");
    auto b = std::make_shared<TreeNode>("b");
    t.add(root);
    t.add(a, root);
    t.add(b, root);

    auto preorder = t.traverse("depthfirst", "preorder");
    auto bfs = t.traverse("breadthfirst", "preorder");

    MINI_CHECK(preorder.size() == 3 && preorder[0]->name == "root");
    MINI_CHECK(bfs.size() == 3 && bfs[0]->name == "root");
}

MINI_TEST("Tree", "Get Node By Name") {
    // uncomment #include "tree.h"
    // uncomment #include "treenode.h"

    Tree t("t");
    auto root = std::make_shared<TreeNode>("root");
    auto child = std::make_shared<TreeNode>("target");
    t.add(root);
    t.add(child, root);

    auto found = t.get_node_by_name("target");

    MINI_CHECK(found != nullptr && found->name == "target");
    MINI_CHECK(t.get_node_by_name("missing") == nullptr);
}

MINI_TEST("Tree", "Get Nodes By Name") {
    // uncomment #include "tree.h"
    // uncomment #include "treenode.h"

    Tree t("t");
    auto root = std::make_shared<TreeNode>("root");
    auto a = std::make_shared<TreeNode>("dup");
    auto b = std::make_shared<TreeNode>("dup");
    t.add(root);
    t.add(a, root);
    t.add(b, root);

    auto found = t.get_nodes_by_name("dup");

    MINI_CHECK(found.size() == 2);
}

MINI_TEST("Tree", "Find Node By Guid") {
    // uncomment #include "tree.h"
    // uncomment #include "treenode.h"

    Tree t("t");
    auto root = std::make_shared<TreeNode>("root");
    t.add(root);
    std::string root_guid = root->guid();

    auto found = t.find_node_by_guid(root_guid);

    MINI_CHECK(found != nullptr && found->guid() == root_guid);
    MINI_CHECK(t.find_node_by_guid("missing-guid") == nullptr);
}

MINI_TEST("Tree", "Add Child By Guid") {
    // uncomment #include "tree.h"
    // uncomment #include "treenode.h"

    Tree t("t");
    auto root = std::make_shared<TreeNode>("root");
    auto a = std::make_shared<TreeNode>("a");
    auto b = std::make_shared<TreeNode>("b");
    t.add(root);
    t.add(a, root);
    t.add(b, root);
    bool ok = t.add_child_by_guid(a->guid(), b->guid());

    MINI_CHECK(ok);
    MINI_CHECK(a->children().size() == 1);
}

MINI_TEST("Tree", "Get Children Guids") {
    // uncomment #include "tree.h"
    // uncomment #include "treenode.h"

    Tree t("t");
    auto root = std::make_shared<TreeNode>("root");
    auto a = std::make_shared<TreeNode>("a");
    auto b = std::make_shared<TreeNode>("b");
    t.add(root);
    t.add(a, root);
    t.add(b, root);

    auto guids = t.get_children_guids(root->guid());

    MINI_CHECK(guids.size() == 2);
    MINI_CHECK(guids[0] == a->guid());
    MINI_CHECK(guids[1] == b->guid());
}

} // namespace session_cpp
