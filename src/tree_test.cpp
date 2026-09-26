#include "mini_test.h"
#include "tree.h"
#include "color.h"
#include "file_encoders.h"
#include "history.h"

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

MINI_TEST("TreeNode", "Set Dead") {

    std::shared_ptr<TreeNode> p = std::make_shared<TreeNode>("p");
    std::shared_ptr<TreeNode> a = std::make_shared<TreeNode>("a");
    std::shared_ptr<TreeNode> b = std::make_shared<TreeNode>("b");
    std::shared_ptr<TreeNode> c = std::make_shared<TreeNode>("c");
    std::shared_ptr<TreeNode> d = std::make_shared<TreeNode>("d");
    p->add(a);
    p->add(b);
    p->add(c);
    b->add(d);
    b->set_dead(true);
    std::vector<TreeNode*> kids = p->children();
    const std::vector<TreeNode*> ancestors = d->ancestors();

    MINI_CHECK(kids.size() == 2 && kids[0] == a.get() && kids[1] == c.get());
    MINI_CHECK(b->is_dead() && b->parent() == nullptr);
    MINI_CHECK(b->children()[0] == d.get());
    MINI_CHECK(d->parent() == b);
    MINI_CHECK(ancestors.size() == 1 && ancestors[0] == b.get());
    MINI_CHECK(!p->is_leaf());

    a->set_dead(true);
    c->set_dead(true);

    MINI_CHECK(p->is_leaf());

    a->set_dead(false);
    b->set_dead(false);
    c->set_dead(false);
    kids = p->children();

    MINI_CHECK(kids.size() == 3 && kids[1] == b.get());
    MINI_CHECK(b->parent() == p);
}

MINI_TEST("TreeNode", "Compact") {

    std::shared_ptr<TreeNode> p = std::make_shared<TreeNode>("p");
    std::vector<std::shared_ptr<TreeNode>> kids;

    for (size_t i = 0; i < 6; ++i) {
        kids.push_back(std::make_shared<TreeNode>("c" + std::to_string(i)));
        p->add(kids[i]);
    }

    const std::shared_ptr<Tomb> tomb = std::make_shared<Tomb>("", false, 0, kids[3]);
    kids[1]->set_dead(true);
    kids[3]->set_dead(true);
    kids[4]->set_dead(true);
    kids[3]->set_tomb(tomb);
    const std::vector<TreeNode*> before = p->children();
    p->compact();
    const std::vector<TreeNode*> after = p->children();
    const size_t raw = p->compact_step(SIZE_MAX);
    std::shared_ptr<TreeNode> q = std::make_shared<TreeNode>("q");
    q->add(kids[5]);
    const std::vector<TreeNode*> moved = p->children();

    MINI_CHECK(raw == 4 && !p->is_compacting());
    MINI_CHECK(after.size() == 3 && after == before);
    MINI_CHECK(kids[3]->is_dead() && kids[3]->get_tomb() != nullptr);
    MINI_CHECK(kids[1]->get_tomb() == nullptr);
    MINI_CHECK(moved.size() == 2 && moved[0] == kids[0].get() && moved[1] == kids[2].get());
}

MINI_TEST("TreeNode", "Add Moves") {

    Tree tree("t");
    std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>("root");
    std::shared_ptr<TreeNode> p1 = std::make_shared<TreeNode>("p1");
    std::shared_ptr<TreeNode> p2 = std::make_shared<TreeNode>("p2");
    std::shared_ptr<TreeNode> w = std::make_shared<TreeNode>("w");
    std::shared_ptr<TreeNode> x = std::make_shared<TreeNode>("x");
    std::shared_ptr<TreeNode> z = std::make_shared<TreeNode>("z");
    std::shared_ptr<TreeNode> y = std::make_shared<TreeNode>("y");
    tree.add(root);
    tree.add(p1, root);
    tree.add(p2, root);
    tree.add(w, p1);
    tree.add(x, p1);
    tree.add(z, p1);
    tree.add(y, x);
    const size_t count = tree.nodes().size();
    const std::shared_ptr<TreeNode> ghost = p2->add(x);
    const std::shared_ptr<TreeNode> again = p2->add(x);
    const std::vector<TreeNode*> old = p1->children();
    const std::vector<TreeNode*> added = p2->children();

    MINI_CHECK(ghost->is_dead() && ghost->name.empty());
    MINI_CHECK(old.size() == 2 && old[0] == w.get() && old[1] == z.get());
    MINI_CHECK(added.back() == x.get());
    MINI_CHECK(x->parent() == p2);
    MINI_CHECK(y->parent() == x);
    MINI_CHECK(tree.nodes().size() == count);
    MINI_CHECK(again == nullptr && added.size() == 1);
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

MINI_TEST("Tree", "Dead Nodes") {

    Tree tree("t");
    std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>("root");
    std::shared_ptr<TreeNode> g = std::make_shared<TreeNode>("group");
    std::shared_ptr<TreeNode> a = std::make_shared<TreeNode>("alpha");
    std::shared_ptr<TreeNode> b = std::make_shared<TreeNode>("beta");
    std::shared_ptr<TreeNode> c = std::make_shared<TreeNode>("gamma");
    std::shared_ptr<TreeNode> d = std::make_shared<TreeNode>("delta");
    tree.add(root);
    tree.add(g, root);
    tree.add(a, g);
    tree.add(b, g);
    tree.add(c, b);
    tree.add(d, g);
    b->set_dead(true);
    const std::string b_guid = b->guid();
    const std::string c_guid = c->guid();
    const std::string g_guid = g->guid();
    const std::string json = tree.jsondump().dump();
    const Tree from_json = Tree::file_json_loads(json);
    const Tree from_pb = Tree::pb_loads(tree.pb_dumps());
    const std::vector<std::string> expected{"root", "group", "alpha", "delta"};
    const std::vector<std::vector<std::shared_ptr<TreeNode>>> orders{
        tree.nodes(),
        tree.traverse("depthfirst", "preorder"),
        tree.traverse("breadthfirst", "preorder"),
        from_json.nodes(),
        from_pb.nodes(),
    };
    bool ordered = true;

    for (const std::vector<std::shared_ptr<TreeNode>>& nodes : orders) {
        std::vector<std::string> names;

        for (const std::shared_ptr<TreeNode>& node : nodes)
            names.push_back(node->name);

        ordered = ordered && names == expected;
    }

    std::vector<std::string> leaves;

    for (const std::shared_ptr<TreeNode>& node : tree.leaves())
        leaves.push_back(node->name);

    MINI_CHECK(ordered);
    MINI_CHECK((leaves == std::vector<std::string>{"alpha", "delta"}));
    MINI_CHECK(tree.get_node_by_name("beta") == nullptr && tree.get_nodes_by_name("gamma").empty());
    MINI_CHECK(tree.find_node_by_guid(b_guid) == nullptr && tree.find_node_by_guid(c_guid) == nullptr);
    MINI_CHECK(tree.get_children_guids(g_guid).size() == 2);
    MINI_CHECK(tree.str().find("beta") == std::string::npos && tree.str().find("gamma") == std::string::npos);
    MINI_CHECK(tree.repr() == "Tree(t, 4 nodes)" && g->str() == "TreeNode(group, 2 children)");
    MINI_CHECK(json.find("beta") == std::string::npos && json.find("gamma") == std::string::npos);
}

} // namespace session_cpp
