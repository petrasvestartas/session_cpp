#include "mini_test.h"
#include "tree.h"
#include "color.h"
#include "file_encoders.h"

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("TreeNode", "Constructor") {
    // uncomment #include "tree.h"
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
    // uncomment #include "tree.h"
    // uncomment #include "file_encoders.h"

    auto original = std::make_shared<TreeNode>("test_node");
    auto child = std::make_shared<TreeNode>("child_node");
    original->add(child);

    std::string fname = "serialization/test_treenode.json";
    file_encoders::file_json_dump(original->jsondump(), fname);
    auto loaded_json = file_encoders::file_json_load_data(fname);
    auto loaded = TreeNode::jsonload(loaded_json);

    MINI_CHECK(loaded->name == original->name);
    MINI_CHECK(loaded->children().size() == 1);
    MINI_CHECK(loaded->children()[0]->name == "child_node");
}

MINI_TEST("TreeNode", "Is Root") {
    // uncomment #include "tree.h"

    auto root = std::make_shared<TreeNode>("root");
    auto child = std::make_shared<TreeNode>("child");
    root->add(child);

    MINI_CHECK(root->is_root());
    MINI_CHECK(!child->is_root());
}

MINI_TEST("TreeNode", "Is Leaf") {
    // uncomment #include "tree.h"

    auto parent = std::make_shared<TreeNode>("parent");
    auto child = std::make_shared<TreeNode>("child");
    parent->add(child);

    MINI_CHECK(child->is_leaf());
    MINI_CHECK(!parent->is_leaf());
}

MINI_TEST("TreeNode", "Tree") {
    // uncomment #include "tree.h"

    auto n = std::make_shared<TreeNode>("standalone");

    MINI_CHECK(n->tree() == nullptr);
}

MINI_TEST("TreeNode", "Add") {
    // uncomment #include "tree.h"

    auto parent = std::make_shared<TreeNode>("parent");
    auto child = std::make_shared<TreeNode>("child");
    parent->add(child);

    MINI_CHECK(parent->children().size() == 1);
    MINI_CHECK(child->parent().get() == parent.get());
}

MINI_TEST("TreeNode", "Remove") {
    // uncomment #include "tree.h"

    auto parent = std::make_shared<TreeNode>("parent");
    auto child = std::make_shared<TreeNode>("child");
    parent->add(child);
    auto removed = parent->remove(child);

    MINI_CHECK(removed == child);
    MINI_CHECK(parent->children().empty());
    MINI_CHECK(child->parent() == nullptr);
}

MINI_TEST("TreeNode", "Parent") {
    // uncomment #include "tree.h"

    auto root = std::make_shared<TreeNode>("root");
    auto child = std::make_shared<TreeNode>("child");
    root->add(child);

    MINI_CHECK(root->parent() == nullptr);
    MINI_CHECK(child->parent().get() == root.get());
}

MINI_TEST("TreeNode", "Ancestors") {
    // uncomment #include "tree.h"

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
    // uncomment #include "tree.h"

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
    // uncomment #include "tree.h"

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
    // uncomment #include "tree.h"

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

    Tree original("test_tree");
    auto root_node = std::make_shared<TreeNode>("root_node");
    original.add(root_node);

    //   jsondump()      │ ordered_json │ to JSON object (internal use)
    //   jsonload(j)     │ ordered_json │ from JSON object (internal use)
    //   file_json_dumps()    │ std::string  │ to JSON string
    //   file_json_loads(s)   │ std::string  │ from JSON string
    //   file_json_dump(path) │ file         │ write to file
    //   file_json_load(path) │ file         │ read from file

    std::string fname = "serialization/test_tree.json";
    original.file_json_dump(fname);
    Tree loaded = Tree::file_json_load(fname);

    MINI_CHECK(loaded.name == original.name);
    MINI_CHECK(loaded.nodes().size() == original.nodes().size());
}

MINI_TEST("Tree", "Protobuf Roundtrip") {
    // uncomment #include "tree.h"

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

    Tree t("t");
    auto root = std::make_shared<TreeNode>("root");
    t.add(root);

    MINI_CHECK(t.root() == root);
}

MINI_TEST("Tree", "Add") {
    // uncomment #include "tree.h"

    Tree t("t");
    auto root = std::make_shared<TreeNode>("root");
    auto child = std::make_shared<TreeNode>("child");
    t.add(root);
    t.add(child, root);

    MINI_CHECK(t.nodes().size() == 2);
}

MINI_TEST("Tree", "Nodes") {
    // uncomment #include "tree.h"

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
