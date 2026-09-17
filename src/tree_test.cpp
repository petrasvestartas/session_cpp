#include "mini_test.h"
#include "tree.h"
#include "color.h"
#include "file_encoders.h"

namespace session_cpp {
using namespace session_cpp::mini_test;

// ═══════════════════════════════════════════════════════════════════════════
// TreeNode
// ═══════════════════════════════════════════════════════════════════════════

MINI_TEST("TreeNode", "Constructor") {

    std::shared_ptr<TreeNode> n0 = std::make_shared<TreeNode>();
    std::shared_ptr<TreeNode> n = std::make_shared<TreeNode>("my_named_node");
    n->color = Color(1.0f, 0.0f, 0.0f, 1.0f);
    std::string nstr = n->str();
    std::shared_ptr<TreeNode> nother = std::make_shared<TreeNode>("my_named_node");

    MINI_CHECK(n0->name == "my_node");
    MINI_CHECK(!n0->guid().empty());
    MINI_CHECK(n->name == "my_named_node");
    MINI_CHECK(n->color.has_value() && n->color->r == 1.0f);
    MINI_CHECK(nstr.find("TreeNode(my_named_node") != std::string::npos);
    MINI_CHECK(*n == *n);
    MINI_CHECK(*n != *nother);
}

MINI_TEST("TreeNode", "Json Roundtrip") {

    std::shared_ptr<TreeNode> original = std::make_shared<TreeNode>("test_node");
    std::shared_ptr<TreeNode> child = std::make_shared<TreeNode>("child_node");
    original->add(child);

    std::string fname = "serialization/test_treenode.json";
    file_encoders::file_json_dump(original->jsondump(), fname);
    std::shared_ptr<TreeNode> loaded = TreeNode::jsonload(file_encoders::file_json_load_data(fname));

    MINI_CHECK(loaded->name == original->name);
    MINI_CHECK(loaded->children().size() == 1);
    MINI_CHECK(loaded->children()[0]->name == "child_node");
}

MINI_TEST("TreeNode", "Is Root") {

    std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>("root");
    std::shared_ptr<TreeNode> child = std::make_shared<TreeNode>("child");
    root->add(child);

    MINI_CHECK(root->is_root());
    MINI_CHECK(!child->is_root());
}

MINI_TEST("TreeNode", "Is Leaf") {

    std::shared_ptr<TreeNode> parent = std::make_shared<TreeNode>("parent");
    std::shared_ptr<TreeNode> child = std::make_shared<TreeNode>("child");
    parent->add(child);

    MINI_CHECK(child->is_leaf());
    MINI_CHECK(!parent->is_leaf());
}

MINI_TEST("TreeNode", "Add") {

    std::shared_ptr<TreeNode> parent = std::make_shared<TreeNode>("parent");
    std::shared_ptr<TreeNode> child = std::make_shared<TreeNode>("child");
    parent->add(child);
    parent->add(parent);

    MINI_CHECK(parent->children().size() == 1);
    MINI_CHECK(child->parent().get() == parent.get());
}

MINI_TEST("TreeNode", "Remove") {

    std::shared_ptr<TreeNode> parent = std::make_shared<TreeNode>("parent");
    std::shared_ptr<TreeNode> child = std::make_shared<TreeNode>("child");
    parent->add(child);
    std::shared_ptr<TreeNode> removed = parent->remove(child);

    MINI_CHECK(removed == child);
    MINI_CHECK(parent->children().empty());
    MINI_CHECK(child->parent() == nullptr);
}

MINI_TEST("TreeNode", "Parent") {

    std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>("root");
    std::shared_ptr<TreeNode> child = std::make_shared<TreeNode>("child");
    root->add(child);

    MINI_CHECK(root->parent() == nullptr);
    MINI_CHECK(child->parent().get() == root.get());
}

MINI_TEST("TreeNode", "Ancestors") {

    std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>("root");
    std::shared_ptr<TreeNode> mid = std::make_shared<TreeNode>("mid");
    std::shared_ptr<TreeNode> leaf = std::make_shared<TreeNode>("leaf");
    root->add(mid);
    mid->add(leaf);

    std::vector<TreeNode*> anc = leaf->ancestors();

    MINI_CHECK(anc.size() == 2);
    MINI_CHECK(anc[0]->name == "mid");
    MINI_CHECK(anc[1]->name == "root");
}

MINI_TEST("TreeNode", "Descendants") {

    std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>("root");
    std::shared_ptr<TreeNode> mid = std::make_shared<TreeNode>("mid");
    std::shared_ptr<TreeNode> leaf = std::make_shared<TreeNode>("leaf");
    root->add(mid);
    mid->add(leaf);

    std::vector<TreeNode*> desc = root->descendants();

    MINI_CHECK(desc.size() == 2);
    MINI_CHECK(desc[0]->name == "mid");
    MINI_CHECK(desc[1]->name == "leaf");
}

MINI_TEST("TreeNode", "Children") {

    std::shared_ptr<TreeNode> parent = std::make_shared<TreeNode>("parent");
    std::shared_ptr<TreeNode> c1 = std::make_shared<TreeNode>("c1");
    std::shared_ptr<TreeNode> c2 = std::make_shared<TreeNode>("c2");
    parent->add(c1);
    parent->add(c2);

    std::vector<TreeNode*> kids = parent->children();

    MINI_CHECK(kids.size() == 2);
    MINI_CHECK(kids[0]->name == "c1");
    MINI_CHECK(kids[1]->name == "c2");
}

MINI_TEST("TreeNode", "Traverse") {

    std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>("root");
    std::shared_ptr<TreeNode> a = std::make_shared<TreeNode>("a");
    std::shared_ptr<TreeNode> b = std::make_shared<TreeNode>("b");
    root->add(a);
    root->add(b);

    std::vector<TreeNode*> preorder = root->traverse("depthfirst", "preorder");
    std::vector<TreeNode*> postorder = root->traverse("depthfirst", "postorder");
    std::vector<TreeNode*> bfs = root->traverse("breadthfirst", "preorder");

    MINI_CHECK(preorder.size() == 3 && preorder[0]->name == "root");
    MINI_CHECK(postorder.size() == 3 && postorder[2]->name == "root");
    MINI_CHECK(bfs.size() == 3 && bfs[0]->name == "root");
}

// ═══════════════════════════════════════════════════════════════════════════
// Tree
// ═══════════════════════════════════════════════════════════════════════════

MINI_TEST("Tree", "Constructor") {

    Tree t0;
    Tree t("my_named_tree");
    std::string tstr = t.str();

    MINI_CHECK(t0.name == "my_tree");
    MINI_CHECK(!t0.guid().empty());
    MINI_CHECK(t.name == "my_named_tree");
    MINI_CHECK(tstr.find("Tree") != std::string::npos);
}

MINI_TEST("Tree", "Json Roundtrip") {

    Tree original("test_tree");
    std::shared_ptr<TreeNode> root_node = std::make_shared<TreeNode>("root_node");
    original.add(root_node);

    std::string fname = "serialization/test_tree.json";
    original.file_json_dump(fname);
    Tree loaded = Tree::file_json_load(fname);

    MINI_CHECK(loaded.name == original.name);
    MINI_CHECK(loaded.nodes().size() == original.nodes().size());
}

MINI_TEST("Tree", "Protobuf Roundtrip") {

    Tree original("test_tree");
    std::shared_ptr<TreeNode> root_node = std::make_shared<TreeNode>("root_node");
    original.add(root_node);

    std::string fname = "serialization/test_tree.bin";
    original.pb_dump(fname);
    Tree loaded = Tree::pb_load(fname);

    MINI_CHECK(loaded.name == original.name);
    MINI_CHECK(loaded.nodes().size() == original.nodes().size());
}

MINI_TEST("Tree", "Root") {

    Tree t("t");
    std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>("root");
    t.add(root);

    MINI_CHECK(t.root() == root);
}

MINI_TEST("Tree", "Add") {

    Tree t("t");
    std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>("root");
    std::shared_ptr<TreeNode> child = std::make_shared<TreeNode>("child");
    t.add(root);
    t.add(child, root);

    MINI_CHECK(t.nodes().size() == 2);
}

MINI_TEST("Tree", "Nodes") {

    Tree t("t");
    std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>("root");
    std::shared_ptr<TreeNode> child = std::make_shared<TreeNode>("child");
    t.add(root);
    t.add(child, root);

    std::vector<std::shared_ptr<TreeNode>> all_nodes = t.nodes();

    MINI_CHECK(all_nodes.size() == 2);
    MINI_CHECK(all_nodes[0]->name == "root");
    MINI_CHECK(all_nodes[1]->name == "child");
}

MINI_TEST("Tree", "Remove") {

    Tree t("t");
    std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>("root");
    std::shared_ptr<TreeNode> child = std::make_shared<TreeNode>("child");
    t.add(root);
    t.add(child, root);
    t.remove(child);

    MINI_CHECK(t.nodes().size() == 1);
}

MINI_TEST("Tree", "Leaves") {

    Tree t("t");
    std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>("root");
    std::shared_ptr<TreeNode> a = std::make_shared<TreeNode>("a");
    std::shared_ptr<TreeNode> b = std::make_shared<TreeNode>("b");
    t.add(root);
    t.add(a, root);
    t.add(b, root);

    std::vector<std::shared_ptr<TreeNode>> lvs = t.leaves();

    MINI_CHECK(lvs.size() == 2);
    MINI_CHECK(lvs[0]->name == "a");
    MINI_CHECK(lvs[1]->name == "b");
}

MINI_TEST("Tree", "Traverse") {

    Tree t("t");
    std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>("root");
    std::shared_ptr<TreeNode> a = std::make_shared<TreeNode>("a");
    std::shared_ptr<TreeNode> b = std::make_shared<TreeNode>("b");
    t.add(root);
    t.add(a, root);
    t.add(b, root);

    std::vector<std::shared_ptr<TreeNode>> preorder = t.traverse("depthfirst", "preorder");
    std::vector<std::shared_ptr<TreeNode>> bfs = t.traverse("breadthfirst", "preorder");

    MINI_CHECK(preorder.size() == 3 && preorder[0]->name == "root");
    MINI_CHECK(bfs.size() == 3 && bfs[0]->name == "root");
}

MINI_TEST("Tree", "Get Node By Name") {

    Tree t("t");
    std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>("root");
    std::shared_ptr<TreeNode> child = std::make_shared<TreeNode>("target");
    t.add(root);
    t.add(child, root);

    std::shared_ptr<TreeNode> found = t.get_node_by_name("target");

    MINI_CHECK(found != nullptr && found->name == "target");
    MINI_CHECK(t.get_node_by_name("missing") == nullptr);
}

MINI_TEST("Tree", "Get Nodes By Name") {

    Tree t("t");
    std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>("root");
    std::shared_ptr<TreeNode> a = std::make_shared<TreeNode>("dup");
    std::shared_ptr<TreeNode> b = std::make_shared<TreeNode>("dup");
    t.add(root);
    t.add(a, root);
    t.add(b, root);

    std::vector<std::shared_ptr<TreeNode>> found = t.get_nodes_by_name("dup");

    MINI_CHECK(found.size() == 2);
}

MINI_TEST("Tree", "Find Node By Guid") {

    Tree t("t");
    std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>("root");
    t.add(root);
    std::string root_guid = root->guid();

    std::shared_ptr<TreeNode> found = t.find_node_by_guid(root_guid);

    MINI_CHECK(found != nullptr && found->guid() == root_guid);
    MINI_CHECK(t.find_node_by_guid("missing-guid") == nullptr);
}

MINI_TEST("Tree", "Add Child By Guid") {

    Tree t("t");
    std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>("root");
    std::shared_ptr<TreeNode> a = std::make_shared<TreeNode>("a");
    std::shared_ptr<TreeNode> b = std::make_shared<TreeNode>("b");
    t.add(root);
    t.add(a, root);
    t.add(b, root);
    bool ok = t.add_child_by_guid(a->guid(), b->guid());
    bool cycle = t.add_child_by_guid(b->guid(), a->guid());

    MINI_CHECK(ok);
    MINI_CHECK(!cycle);
    MINI_CHECK(a->children().size() == 1);
}

MINI_TEST("Tree", "Get Children Guids") {

    Tree t("t");
    std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>("root");
    std::shared_ptr<TreeNode> a = std::make_shared<TreeNode>("a");
    std::shared_ptr<TreeNode> b = std::make_shared<TreeNode>("b");
    t.add(root);
    t.add(a, root);
    t.add(b, root);

    std::vector<std::string> guids = t.get_children_guids(root->guid());

    MINI_CHECK(guids.size() == 2);
    MINI_CHECK(guids[0] == a->guid());
    MINI_CHECK(guids[1] == b->guid());
}

} // namespace session_cpp
