#include "mini_test.h"
#include "treenode.h"
#include "color.h"
#include "encoders.h"

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("TreeNode", "Constructor") {
    // uncomment #include "treenode.h"
    // uncomment #include "color.h"

    // Default constructor
    auto n0 = std::make_shared<TreeNode>();

    // Constructor with name
    auto n = std::make_shared<TreeNode>("my_named_node");
    n->color = Color(255, 0, 0, 255);

    // Minimal string representation
    std::string nstr = n->str();

    // Copies (compared by guid)
    auto nother = std::make_shared<TreeNode>("my_named_node");

    MINI_CHECK(n0->name == "my_node");
    MINI_CHECK(!n0->guid().empty());
    MINI_CHECK(n->name == "my_named_node");
    MINI_CHECK(n->color.has_value() && n->color->r == 255);
    MINI_CHECK(nstr.find("TreeNode(my_named_node") != std::string::npos);
    MINI_CHECK(*n == *n);
    MINI_CHECK(*n != *nother);
}

MINI_TEST("TreeNode", "Json Roundtrip") {
    // uncomment #include "treenode.h"
    // uncomment #include "encoders.h"

    auto original = std::make_shared<TreeNode>("test_node");
    auto child = std::make_shared<TreeNode>("child_node");
    original->add(child);

    std::string fname = "serialization/test_treenode.json";
    encoders::json_dump(original->jsondump(), fname);
    auto loaded_json = encoders::json_load_data(fname);
    auto loaded = TreeNode::jsonload(loaded_json);

    MINI_CHECK(loaded->name == original->name);
    MINI_CHECK(loaded->children().size() == 1);
    MINI_CHECK(loaded->children()[0]->name == "child_node");
}

MINI_TEST("TreeNode", "Is Root") {
    // uncomment #include "treenode.h"

    auto root = std::make_shared<TreeNode>("root");
    auto child = std::make_shared<TreeNode>("child");
    root->add(child);

    MINI_CHECK(root->is_root());
    MINI_CHECK(!child->is_root());
}

MINI_TEST("TreeNode", "Is Leaf") {
    // uncomment #include "treenode.h"

    auto parent = std::make_shared<TreeNode>("parent");
    auto child = std::make_shared<TreeNode>("child");
    parent->add(child);

    MINI_CHECK(child->is_leaf());
    MINI_CHECK(!parent->is_leaf());
}

MINI_TEST("TreeNode", "Tree") {
    // uncomment #include "treenode.h"

    auto n = std::make_shared<TreeNode>("standalone");

    MINI_CHECK(n->tree() == nullptr);
}

MINI_TEST("TreeNode", "Add") {
    // uncomment #include "treenode.h"

    auto parent = std::make_shared<TreeNode>("parent");
    auto child = std::make_shared<TreeNode>("child");
    parent->add(child);

    MINI_CHECK(parent->children().size() == 1);
    MINI_CHECK(child->parent().get() == parent.get());
}

MINI_TEST("TreeNode", "Remove") {
    // uncomment #include "treenode.h"

    auto parent = std::make_shared<TreeNode>("parent");
    auto child = std::make_shared<TreeNode>("child");
    parent->add(child);
    auto removed = parent->remove(child);

    MINI_CHECK(removed == child);
    MINI_CHECK(parent->children().empty());
    MINI_CHECK(child->parent() == nullptr);
}

MINI_TEST("TreeNode", "Parent") {
    // uncomment #include "treenode.h"

    auto root = std::make_shared<TreeNode>("root");
    auto child = std::make_shared<TreeNode>("child");
    root->add(child);

    MINI_CHECK(root->parent() == nullptr);
    MINI_CHECK(child->parent().get() == root.get());
}

MINI_TEST("TreeNode", "Ancestors") {
    // uncomment #include "treenode.h"

    auto root = std::make_shared<TreeNode>("root");
    auto mid = std::make_shared<TreeNode>("mid");
    auto leaf = std::make_shared<TreeNode>("leaf");
    root->add(mid);
    mid->add(leaf);

    auto anc = leaf->ancestors();

    MINI_CHECK(anc.size() == 2);
    MINI_CHECK(anc[0]->name == "mid");
    MINI_CHECK(anc[1]->name == "root");
}

MINI_TEST("TreeNode", "Descendants") {
    // uncomment #include "treenode.h"

    auto root = std::make_shared<TreeNode>("root");
    auto mid = std::make_shared<TreeNode>("mid");
    auto leaf = std::make_shared<TreeNode>("leaf");
    root->add(mid);
    mid->add(leaf);

    auto desc = root->descendants();

    MINI_CHECK(desc.size() == 2);
    MINI_CHECK(desc[0]->name == "mid");
    MINI_CHECK(desc[1]->name == "leaf");
}

MINI_TEST("TreeNode", "Children") {
    // uncomment #include "treenode.h"

    auto parent = std::make_shared<TreeNode>("parent");
    auto c1 = std::make_shared<TreeNode>("c1");
    auto c2 = std::make_shared<TreeNode>("c2");
    parent->add(c1);
    parent->add(c2);

    auto kids = parent->children();

    MINI_CHECK(kids.size() == 2);
    MINI_CHECK(kids[0]->name == "c1");
    MINI_CHECK(kids[1]->name == "c2");
}

MINI_TEST("TreeNode", "Traverse") {
    // uncomment #include "treenode.h"

    auto root = std::make_shared<TreeNode>("root");
    auto a = std::make_shared<TreeNode>("a");
    auto b = std::make_shared<TreeNode>("b");
    root->add(a);
    root->add(b);

    auto preorder = root->traverse("depthfirst", "preorder");
    auto postorder = root->traverse("depthfirst", "postorder");
    auto bfs = root->traverse("breadthfirst", "preorder");

    MINI_CHECK(preorder.size() == 3 && preorder[0]->name == "root");
    MINI_CHECK(postorder.size() == 3 && postorder[2]->name == "root");
    MINI_CHECK(bfs.size() == 3 && bfs[0]->name == "root");
}

} // namespace session_cpp
