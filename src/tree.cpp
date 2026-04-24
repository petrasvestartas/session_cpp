#include "tree.h"
#include "tree.pb.h"
#include "treenode.pb.h"
#include "fmt/format.h"
#include <algorithm>
#include <iostream>
#include <queue>
#include <sstream>
#include <stdexcept>

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// TreeNode: JSON
///////////////////////////////////////////////////////////////////////////////////////////

/// Convert to JSON-serializable object
nlohmann::ordered_json TreeNode::jsondump() const {

  nlohmann::ordered_json children_array = nlohmann::ordered_json::array();
  for (const auto &child : _children)
    children_array.push_back(child->jsondump());

  nlohmann::ordered_json j{{"type", "TreeNode"},
                           {"guid", guid()},
                           {"name", name},
                           {"children", children_array}};
  if (color) j["color"] = color->jsondump();
  return j;
}

/// Create TreeNode from JSON data
std::shared_ptr<TreeNode> TreeNode::jsonload(const nlohmann::json &data) {
  auto node = std::make_shared<TreeNode>(data["name"]);
  node->guid() = data["guid"];
  if (data.contains("color") && !data["color"].is_null())
    node->color = Color::jsonload(data["color"]);
  for (const auto &child_data : data["children"]) {
    auto child = TreeNode::jsonload(child_data);
    node->add(child);
  }
  return node;
}

///////////////////////////////////////////////////////////////////////////////////////////
// TreeNode Implementation
///////////////////////////////////////////////////////////////////////////////////////////

std::string TreeNode::str() const {
  return fmt::format("TreeNode({}, {}, {} children)", this->name, this->guid(),
                     _children.size());
}

bool TreeNode::operator==(const TreeNode &other) const {
  return this->guid() == other.guid();
}

bool TreeNode::operator!=(const TreeNode &other) const {
  return !(*this == other);
}

bool TreeNode::is_root() const { return _parent.expired(); }

bool TreeNode::is_leaf() const { return _children.empty(); }

std::shared_ptr<TreeNode> TreeNode::parent() const { return _parent.lock(); }

std::vector<TreeNode *> TreeNode::children() const {
  std::vector<TreeNode *> result;
  for (const auto &child : _children) {
    result.push_back(child.get());
  }
  return result;
}

Tree *TreeNode::tree() const { return _tree.lock().get(); }

void TreeNode::add(std::shared_ptr<TreeNode> child) {
  if (!child)
    return;

  // Always require shared_from_this() to work - this enforces proper shared_ptr
  // usage
  child->_parent = shared_from_this();
  child->_tree = _tree;
  _children.push_back(child);
}

std::shared_ptr<TreeNode> TreeNode::remove(std::shared_ptr<TreeNode> child) {
  auto it = std::find_if(
      _children.begin(), _children.end(),
      [child](const std::shared_ptr<TreeNode> &ptr) { return ptr == child; });

  if (it != _children.end()) {
    auto removed = *it;
    _children.erase(it);
    removed->_parent.reset();
    return removed;
  }
  return nullptr;
}

std::vector<TreeNode *> TreeNode::ancestors() const {
  std::vector<TreeNode *> result;
  auto current = _parent.lock();
  while (current) {
    result.push_back(current.get());
    current = current->_parent.lock();
  }
  return result;
}

std::vector<TreeNode *> TreeNode::descendants() const {
  std::vector<TreeNode *> result;
  for (const auto &child : _children) {
    result.push_back(child.get());
    auto child_descendants = child->descendants();
    result.insert(result.end(), child_descendants.begin(),
                  child_descendants.end());
  }
  return result;
}

std::vector<TreeNode *> TreeNode::traverse(const std::string &strategy,
                                           const std::string &order) const {
  std::vector<TreeNode *> result;

  if (strategy == "depthfirst") {
    if (order == "preorder") {
      result.push_back(const_cast<TreeNode *>(this));
      for (const auto &child : _children) {
        auto child_result = child->traverse(strategy, order);
        result.insert(result.end(), child_result.begin(), child_result.end());
      }
    } else if (order == "postorder") {
      for (const auto &child : _children) {
        auto child_result = child->traverse(strategy, order);
        result.insert(result.end(), child_result.begin(), child_result.end());
      }
      result.push_back(const_cast<TreeNode *>(this));
    }
  } else if (strategy == "breadthfirst") {
    std::queue<TreeNode *> queue;
    queue.push(const_cast<TreeNode *>(this));
    while (!queue.empty()) {
      auto current = queue.front();
      queue.pop();
      result.push_back(current);
      for (const auto &child : current->_children) {
        queue.push(child.get());
      }
    }
  } else {
    throw std::invalid_argument("Unknown traversal strategy: " + strategy);
  }

  return result;
}

std::ostream &operator<<(std::ostream &os, const TreeNode &node) {
  os << node.str();
  return os;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Tree Implementation
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// JSON
///////////////////////////////////////////////////////////////////////////////////////////

nlohmann::ordered_json Tree::jsondump() const {

  return nlohmann::ordered_json{
      {"type", "Tree"},
      {"guid", guid()},
      {"name", name},
      {"root",
       _root ? _root->jsondump() : nlohmann::ordered_json(nullptr)}};
}

Tree Tree::jsonload(const nlohmann::json &data) {
  auto tree = std::make_shared<Tree>(data["name"]);
  tree->guid() = data["guid"];
  if (!data["root"].is_null()) {
    auto root = TreeNode::jsonload(data["root"]);
    tree->add(root);
  }
  return *tree;
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
  nlohmann::json data = nlohmann::json::parse(file);
  return jsonload(data);
}

std::string Tree::pb_dumps() const {
  std::function<session_proto::TreeNode(TreeNode*)> node_to_proto =
    [&](TreeNode* node) -> session_proto::TreeNode {
      session_proto::TreeNode proto_node;
      proto_node.set_guid(node->guid());
      proto_node.set_name(node->name);
      proto_node.set_parent_guid("");
      if (node->color) {
        auto* pc = proto_node.mutable_color();
        pc->set_r(node->color->r);
        pc->set_g(node->color->g);
        pc->set_b(node->color->b);
        pc->set_a(node->color->a);
      }
      for (auto* child : node->children()) {
        *proto_node.add_children() = node_to_proto(child);
      }
      return proto_node;
    };

  session_proto::Tree proto;
  proto.set_guid(guid());
  proto.set_name(name);
  if (_root) {
    *proto.mutable_root() = node_to_proto(_root.get());
  }
  return proto.SerializeAsString();
}

Tree Tree::pb_loads(const std::string& data) {
  session_proto::Tree proto;
  proto.ParseFromString(data);

  std::function<std::shared_ptr<TreeNode>(const session_proto::TreeNode&)> proto_to_node =
    [&](const session_proto::TreeNode& proto_node) -> std::shared_ptr<TreeNode> {
      auto node = std::make_shared<TreeNode>(proto_node.name());
      node->guid() = proto_node.guid();
      if (proto_node.has_color() && proto_node.color().a() > 0)
        node->color = Color(proto_node.color().r(), proto_node.color().g(),
                            proto_node.color().b(), proto_node.color().a());
      for (const auto& child_proto : proto_node.children()) {
        auto child = proto_to_node(child_proto);
        node->add(child);
      }
      return node;
    };

  Tree tree(proto.name());
  tree.guid() = proto.guid();
  if (proto.has_root()) {
    tree.add(proto_to_node(proto.root()));
  }
  return tree;
}

void Tree::pb_dump(const std::string& filename) const {
  std::string data = pb_dumps();
  std::ofstream file(filename, std::ios::binary);
  file.write(data.data(), data.size());
}

Tree Tree::pb_load(const std::string& filename) {
  std::ifstream file(filename, std::ios::binary);
  std::string data((std::istreambuf_iterator<char>(file)),
                    std::istreambuf_iterator<char>());
  return pb_loads(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Details
///////////////////////////////////////////////////////////////////////////////////////////

std::string Tree::str() const { return fmt::format("Tree: {}", name); }

std::shared_ptr<TreeNode> Tree::root() const { return _root; }

void Tree::add(std::shared_ptr<TreeNode> node,
               std::shared_ptr<TreeNode> parent) {
  if (!node)
    throw std::invalid_argument("Cannot add null node");

  if (!parent) {
    // Add as root
    if (_root)
      throw std::runtime_error("Tree already has a root node");

    _root = node;
    // Set tree reference using raw pointer (since Tree is not shared_ptr
    // managed)
    node->_tree = std::weak_ptr<Tree>();
  } else {
    // Add as child to parent
    node->_tree = std::weak_ptr<Tree>();
    parent->add(node);
  }
}

std::vector<std::shared_ptr<TreeNode>> Tree::nodes() const {
  std::vector<std::shared_ptr<TreeNode>> result;
  if (!_root)
    return result;

  // Use a queue for breadth-first traversal to collect all shared_ptrs
  std::queue<std::shared_ptr<TreeNode>> queue;
  queue.push(_root);

  while (!queue.empty()) {
    auto current = queue.front();
    queue.pop();
    result.push_back(current);

    // Add children to queue
    for (const auto &child : current->_children) {
      queue.push(child);
    }
  }

  return result;
}

std::shared_ptr<TreeNode> Tree::remove(std::shared_ptr<TreeNode> node) {
  if (!node) {
    throw std::invalid_argument("Cannot remove null node");
  }

  // If removing the root
  if (node == _root) {
    auto removed = _root;
    _root.reset();
    removed->_tree.reset();
    return removed;
  } else {
    // Get parent and remove the node from its children
    if (auto parent = node->parent()) {
      return parent->remove(node);
    }
    // If node has no parent, it's not in this tree
    throw std::invalid_argument("Node is not in this tree");
  }
}

std::vector<std::shared_ptr<TreeNode>> Tree::leaves() const {
  std::vector<std::shared_ptr<TreeNode>> result;
  if (_root) {
    for (const auto &node : nodes()) {
      if (node->is_leaf()) {
        result.push_back(node);
      }
    }
  }
  return result;
}

std::vector<std::shared_ptr<TreeNode>>
Tree::traverse(const std::string &strategy, const std::string &order) const {
  std::vector<std::shared_ptr<TreeNode>> result;
  if (!_root)
    return result;

  auto raw_nodes = _root->traverse(strategy, order);
  for (auto raw_node : raw_nodes) {
    // Find the corresponding shared_ptr for each raw pointer
    for (const auto &node : nodes()) {
      if (node.get() == raw_node) {
        result.push_back(node);
        break;
      }
    }
  }
  return result;
}

std::shared_ptr<TreeNode>
Tree::get_node_by_name(const std::string &node_name) const {
  for (const auto &node : nodes()) {
    if (node->name == node_name) {
      return node;
    }
  }
  return nullptr;
}

std::vector<std::shared_ptr<TreeNode>>
Tree::get_nodes_by_name(const std::string &node_name) const {
  std::vector<std::shared_ptr<TreeNode>> result;
  for (const auto &node : nodes()) {
    if (node->name == node_name) {
      result.push_back(node);
    }
  }
  return result;
}

std::shared_ptr<TreeNode>
Tree::find_node_by_guid(const std::string &node_guid) const {
  for (const auto &node : nodes()) {
    if (node->guid() == node_guid) {
      return node;
    }
  }
  return nullptr;
}

bool Tree::add_child_by_guid(const std::string &parent_guid,
                             const std::string &child_guid) {
  auto parent_node = find_node_by_guid(parent_guid);
  auto child_node = find_node_by_guid(child_guid);

  if (!parent_node || !child_node) {
    return false;
  }

  // Remove child from its current parent if it has one
  if (auto current_parent = child_node->parent()) {
    current_parent->remove(child_node);
  } else {
    // Child is not currently in any parent, we can't move it
    return false;
  }

  // Add to new parent
  parent_node->add(child_node);
  return true;
}

std::vector<std::string>
Tree::get_children_guids(const std::string &node_guid) const {
  auto node = find_node_by_guid(node_guid);
  if (!node) {
    return {};
  }

  std::vector<std::string> result;
  for (const auto &child : node->children()) {
    result.push_back(child->guid());
  }
  return result;
}

void Tree::print_hierarchy() const {
  if (!_root) {
    std::cout << "Empty tree" << std::endl;
    return;
  }

  // Lambda function for recursive printing
  std::function<void(TreeNode *, const std::string &, bool)> print_node =
      [&](TreeNode *node, const std::string &prefix, bool is_last) {
        if (!node)
          return;

        // Print current node
        std::cout << prefix << (is_last ? "└── " : "├── ") << node->str()
                  << std::endl;

        // New prefix for children
        std::string new_prefix = prefix + (is_last ? "    " : "│   ");

        // Print children
        const auto &children = node->children();
        for (size_t i = 0; i < children.size(); ++i) {
          bool is_last_child = (i == children.size() - 1);
          print_node(children[i], new_prefix, is_last_child);
        }
      };

  // Start recursive printing from root
  print_node(_root.get(), "", true);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Not class methods
///////////////////////////////////////////////////////////////////////////////////////////

std::ostream &operator<<(std::ostream &os, const Tree &tree) {
  os << tree.str();
  return os;
}

} // namespace session_cpp
