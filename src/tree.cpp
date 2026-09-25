#include "tree.h"
#include "tree.pb.h"
#include "treenode.pb.h"
#include "fmt/format.h"
#include <algorithm>
#include <fstream>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// TreeNode
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// Accessors
// ═══════════════════════════════════════════════════════════════════════════
bool TreeNode::is_root() const {
    return _parent.expired();
}

bool TreeNode::is_leaf() const {
    return _children.empty();
}

std::shared_ptr<TreeNode> TreeNode::parent() const {
    return _parent.lock();
}

std::vector<TreeNode*> TreeNode::ancestors() const {

    std::vector<TreeNode*> result;
    std::shared_ptr<TreeNode> current = _parent.lock();

    while (current) {
        result.push_back(current.get());
        current = current->_parent.lock();
    }

    return result;
}

std::vector<TreeNode*> TreeNode::descendants() const {

    std::vector<TreeNode*> result = traverse("depthfirst", "preorder");
    result.erase(result.begin());

    return result;
}

std::vector<TreeNode*> TreeNode::children() const {

    std::vector<TreeNode*> result;

    for (const std::shared_ptr<TreeNode>& child : _children)
        result.push_back(child.get());

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Mutators
// ═══════════════════════════════════════════════════════════════════════════
void TreeNode::add(std::shared_ptr<TreeNode> child) {

    if (!child)
        return;

    if (child.get() == this)
        return;

    for (std::shared_ptr<TreeNode> ancestor = shared_from_this(); ancestor; ancestor = ancestor->parent())
        if (ancestor == child)
            return;

    child->_parent = shared_from_this();
    _children.push_back(child);
}

std::shared_ptr<TreeNode> TreeNode::remove(std::shared_ptr<TreeNode> child) {

    for (size_t i = 0; i < _children.size(); ++i) {
        if (_children[i] != child)
            continue;

        std::shared_ptr<TreeNode> removed = _children[i];
        _children.erase(_children.begin() + i);
        removed->_parent.reset();

        return removed;
    }

    return nullptr;
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
bool TreeNode::operator==(const TreeNode& other) const {
    return guid() == other.guid();
}

bool TreeNode::operator!=(const TreeNode& other) const {
    return !(*this == other);
}

// ═══════════════════════════════════════════════════════════════════════════
// Traversal
// ═══════════════════════════════════════════════════════════════════════════
std::vector<TreeNode*> TreeNode::traverse(const std::string& strategy, const std::string& order) const {

    std::vector<TreeNode*> result;

    if (strategy == "depthfirst") {
        if (order != "preorder" && order != "postorder")
            throw std::invalid_argument("Unknown traversal order: " + order);

        std::vector<TreeNode*> stack{const_cast<TreeNode*>(this)};

        while (!stack.empty()) {
            TreeNode* current = stack.back();
            stack.pop_back();
            result.push_back(current);

            if (order == "preorder")
                for (size_t i = current->_children.size(); i > 0; --i)
                    stack.push_back(current->_children[i - 1].get());
            else
                for (const std::shared_ptr<TreeNode>& child : current->_children)
                    stack.push_back(child.get());
        }

        if (order == "postorder")
            std::reverse(result.begin(), result.end());
    } else if (strategy == "breadthfirst") {
        std::queue<TreeNode*> queue;
        queue.push(const_cast<TreeNode*>(this));

        while (!queue.empty()) {
            TreeNode* current = queue.front();
            queue.pop();
            result.push_back(current);

            for (const std::shared_ptr<TreeNode>& child : current->_children)
                queue.push(child.get());
        }
    } else {
        throw std::invalid_argument("Unknown traversal strategy: " + strategy);
    }

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════
nlohmann::ordered_json TreeNode::jsondump() const {

    nlohmann::ordered_json children = nlohmann::ordered_json::array();

    for (const std::shared_ptr<TreeNode>& child : _children)
        children.push_back(child->jsondump());

    nlohmann::ordered_json data;
    data["children"] = children;

    if (color)
        data["color"] = color->jsondump();

    data["guid"] = guid();
    data["name"] = name;
    data["type"] = "TreeNode";

    return data;
}

std::shared_ptr<TreeNode> TreeNode::jsonload(const nlohmann::json& data) {

    std::shared_ptr<TreeNode> node = std::make_shared<TreeNode>(data["name"]);
    node->guid() = data["guid"];

    if (data.contains("color") && !data["color"].is_null())
        node->color = Color::jsonload(data["color"]);

    for (const nlohmann::json& child : data["children"])
        node->add(TreeNode::jsonload(child));

    return node;
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
std::string TreeNode::str() const {
    return fmt::format("TreeNode({}, {} children)", name, _children.size());
}

std::string TreeNode::repr() const {
    return fmt::format("TreeNode({}, {}, {} children)", name, guid(), _children.size());
}

std::ostream& operator<<(std::ostream& os, const TreeNode& node) {

    os << node.str();

    return os;
}

// ═══════════════════════════════════════════════════════════════════════════
// Tree
// ═══════════════════════════════════════════════════════════════════════════
namespace {

/// Duplicate one node and everything under it with the same names, guids and colours.
std::shared_ptr<TreeNode> clone_node(const TreeNode& node) {

    std::shared_ptr<TreeNode> copy = std::make_shared<TreeNode>(node.name);

    if (node.has_guid())
        copy->guid() = node.guid();

    copy->color = node.color;

    for (const TreeNode* child : node.children())
        copy->add(clone_node(*child));

    return copy;
}

/// Convert a node and its subtree to protobuf.
session_proto::TreeNode node_to_proto(const TreeNode& node) {

    session_proto::TreeNode proto;
    proto.set_guid(node.guid());
    proto.set_name(node.name);
    proto.set_parent_guid("");

    if (node.color) {
        proto.mutable_color()->set_r(node.color->r);
        proto.mutable_color()->set_g(node.color->g);
        proto.mutable_color()->set_b(node.color->b);
        proto.mutable_color()->set_a(node.color->a);
    }

    for (const TreeNode* child : node.children())
        *proto.add_children() = node_to_proto(*child);

    return proto;
}

/// Convert a protobuf node and its subtree to a TreeNode.
std::shared_ptr<TreeNode> proto_to_node(const session_proto::TreeNode& proto) {

    std::shared_ptr<TreeNode> node = std::make_shared<TreeNode>(proto.name());
    node->guid() = proto.guid();

    if (proto.has_color() && proto.color().a() > 0)
        node->color = Color(proto.color().r(), proto.color().g(), proto.color().b(), proto.color().a());

    for (const session_proto::TreeNode& child : proto.children())
        node->add(proto_to_node(child));

    return node;
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
Tree::Tree(const Tree& other) : name(other.name) {

    if (other.has_guid())
        guid() = other.guid();

    if (other._root)
        _root = clone_node(*other._root);
}

Tree& Tree::operator=(const Tree& other) {

    if (this != &other) {
        Tree copy(other);
        *this = std::move(copy);
    }

    return *this;
}

// ═══════════════════════════════════════════════════════════════════════════
// Accessors
// ═══════════════════════════════════════════════════════════════════════════
std::shared_ptr<TreeNode> Tree::root() const {
    return _root;
}

std::vector<std::shared_ptr<TreeNode>> Tree::nodes() const {

    std::vector<std::shared_ptr<TreeNode>> result;

    if (!_root)
        return result;

    std::queue<std::shared_ptr<TreeNode>> queue;
    queue.push(_root);

    while (!queue.empty()) {
        std::shared_ptr<TreeNode> current = queue.front();
        queue.pop();
        result.push_back(current);

        for (const std::shared_ptr<TreeNode>& child : current->_children)
            queue.push(child);
    }

    return result;
}

std::vector<std::shared_ptr<TreeNode>> Tree::leaves() const {

    std::vector<std::shared_ptr<TreeNode>> result;

    for (const std::shared_ptr<TreeNode>& node : nodes())
        if (node->is_leaf())
            result.push_back(node);

    return result;
}

std::shared_ptr<TreeNode> Tree::get_node_by_name(const std::string& node_name) const {

    for (const std::shared_ptr<TreeNode>& node : nodes())
        if (node->name == node_name)
            return node;

    return nullptr;
}

std::vector<std::shared_ptr<TreeNode>> Tree::get_nodes_by_name(const std::string& node_name) const {

    std::vector<std::shared_ptr<TreeNode>> result;

    for (const std::shared_ptr<TreeNode>& node : nodes())
        if (node->name == node_name)
            result.push_back(node);

    return result;
}

std::shared_ptr<TreeNode> Tree::find_node_by_guid(const std::string& node_guid) const {

    for (const std::shared_ptr<TreeNode>& node : nodes())
        if (node->guid() == node_guid)
            return node;

    return nullptr;
}

std::vector<std::string> Tree::get_children_guids(const std::string& node_guid) const {

    std::vector<std::string> result;
    std::shared_ptr<TreeNode> node = find_node_by_guid(node_guid);

    if (!node)
        return result;

    for (const TreeNode* child : node->children())
        result.push_back(child->guid());

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Mutators
// ═══════════════════════════════════════════════════════════════════════════
void Tree::add(std::shared_ptr<TreeNode> node, std::shared_ptr<TreeNode> parent) {

    if (!node)
        throw std::invalid_argument("Cannot add null node");

    if (parent) {
        parent->add(node);

        return;
    }

    if (_root)
        throw std::runtime_error("Tree already has a root node");

    _root = node;
}

std::shared_ptr<TreeNode> Tree::remove(std::shared_ptr<TreeNode> node) {

    if (!node)
        throw std::invalid_argument("Cannot remove null node");

    if (node == _root) {
        _root.reset();

        return node;
    }

    std::shared_ptr<TreeNode> parent = node->parent();

    if (!parent)
        throw std::invalid_argument("Node is not in this tree");

    return parent->remove(node);
}

bool Tree::add_child_by_guid(const std::string& parent_guid, const std::string& child_guid) {

    std::shared_ptr<TreeNode> parent = find_node_by_guid(parent_guid);
    std::shared_ptr<TreeNode> child = find_node_by_guid(child_guid);

    if (!parent || !child)
        return false;

    if (parent == child)
        return false;

    for (std::shared_ptr<TreeNode> ancestor = parent; ancestor; ancestor = ancestor->parent())
        if (ancestor == child)
            return false;

    std::shared_ptr<TreeNode> current = child->parent();

    if (!current)
        return false;

    current->remove(child);
    parent->add(child);

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Traversal
// ═══════════════════════════════════════════════════════════════════════════
std::vector<std::shared_ptr<TreeNode>> Tree::traverse(const std::string& strategy, const std::string& order) const {

    std::vector<std::shared_ptr<TreeNode>> result;

    if (!_root)
        return result;

    std::unordered_map<TreeNode*, std::shared_ptr<TreeNode>> lookup;

    for (const std::shared_ptr<TreeNode>& node : nodes())
        lookup[node.get()] = node;

    for (TreeNode* node : _root->traverse(strategy, order))
        result.push_back(lookup[node]);

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════
nlohmann::ordered_json Tree::jsondump() const {

    nlohmann::ordered_json data;
    data["guid"] = guid();
    data["name"] = name;
    data["root"] = _root ? _root->jsondump() : nlohmann::ordered_json(nullptr);
    data["type"] = "Tree";

    return data;
}

Tree Tree::jsonload(const nlohmann::json& data) {

    Tree tree(data["name"]);
    tree.guid() = data["guid"];

    if (!data["root"].is_null())
        tree.add(TreeNode::jsonload(data["root"]));

    return tree;
}

std::string Tree::file_json_dumps() const {
    return jsondump().dump();
}

Tree Tree::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Tree::file_json_dump(const std::string& filename) const {

    std::ofstream file(filename);
    file << jsondump().dump(4);
}

Tree Tree::file_json_load(const std::string& filename) {

    std::ifstream file(filename);

    return jsonload(nlohmann::json::parse(file));
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════
std::string Tree::pb_dumps() const {

    session_proto::Tree proto;

    if (has_guid())
        proto.set_guid(guid());

    proto.set_name(name);

    if (_root)
        *proto.mutable_root() = node_to_proto(*_root);

    return proto.SerializeAsString();
}

Tree Tree::pb_loads(const std::string& data) {

    session_proto::Tree proto;
    proto.ParseFromString(data);
    Tree tree(proto.name());

    if (!proto.guid().empty())
        tree.guid() = proto.guid();

    if (proto.has_root())
        tree.add(proto_to_node(proto.root()));

    return tree;
}

void Tree::pb_dump(const std::string& filename) const {

    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

Tree Tree::pb_load(const std::string& filename) {

    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
namespace {

/// Draw one node and its subtree, the last child of every level closing its branch.
void draw_node(std::ostringstream& os, const TreeNode& node, const std::string& prefix, bool last) {

    os << prefix << (last ? "\u2514\u2500\u2500 " : "\u251c\u2500\u2500 ") << node.str() << "\n";

    const std::vector<TreeNode*> kids = node.children();
    const std::string next = prefix + (last ? "    " : "\u2502   ");
    for (size_t i = 0; i < kids.size(); ++i)
        draw_node(os, *kids[i], next, i + 1 == kids.size());
}

} // namespace

std::string Tree::str() const {

    std::ostringstream os;
    os << fmt::format("<Tree with {} nodes: {}>\n", nodes().size(), name);

    if (const std::shared_ptr<TreeNode> start = root())
        draw_node(os, *start, "", true);

    return os.str();
}

std::string Tree::repr() const {
    return fmt::format("Tree({}, {} nodes)", name, nodes().size());
}

std::ostream& operator<<(std::ostream& os, const Tree& tree) {

    os << tree.str();

    return os;
}

} // namespace session_cpp
