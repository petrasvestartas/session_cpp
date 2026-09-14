#pragma once
#include "color.h"
#include <guid.h>
#include <json.h>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// TreeNode
// ═══════════════════════════════════════════════════════════════════════════

/// A node of a tree; geometry nodes are named by their object's guid, group nodes by a label
class TreeNode : public std::enable_shared_from_this<TreeNode> {
  friend class Tree;

private:
  mutable std::string _guid;
  std::weak_ptr<TreeNode> _parent;
  std::vector<std::shared_ptr<TreeNode>> _children;

public:
  std::string name;
  std::optional<Color> color;

  TreeNode(std::string name = "my_node") : name(name) {}

  bool has_guid() const { return !_guid.empty(); }
  const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
  std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// True if this node has no parent
  bool is_root() const;

  /// True if this node has no children
  bool is_leaf() const;

  /// Add a child node to this node
  void add(std::shared_ptr<TreeNode> child);

  /// Remove a child node and return it (nullptr if not found)
  std::shared_ptr<TreeNode> remove(std::shared_ptr<TreeNode> child);

  /// Parent node, or nullptr if this is the root
  std::shared_ptr<TreeNode> parent() const;

  /// All ancestors from immediate parent up to root
  std::vector<TreeNode *> ancestors() const;

  /// All descendants of this node, depth-first
  std::vector<TreeNode *> descendants() const;

  /// Direct children of this node
  std::vector<TreeNode *> children() const;

  /// Traverse from this node ("depthfirst"|"breadthfirst", "preorder"|"postorder")
  std::vector<TreeNode *> traverse(const std::string &strategy = "depthfirst", const std::string &order = "preorder") const;

  /// Equality (compares guid)
  bool operator==(const TreeNode &other) const;

  /// Inequality (compares guid)
  bool operator!=(const TreeNode &other) const;

  nlohmann::ordered_json jsondump() const;
  static std::shared_ptr<TreeNode> jsonload(const nlohmann::json &data);

  std::string str() const;
};

std::ostream &operator<<(std::ostream &os, const TreeNode &node);

// ═══════════════════════════════════════════════════════════════════════════
// Tree
// ═══════════════════════════════════════════════════════════════════════════

/// A hierarchy of TreeNodes under one root
class Tree {
private:
  mutable std::string _guid;
  std::shared_ptr<TreeNode> _root;

public:
  std::string name;

  Tree(std::string name = "my_tree") : name(name) {}

  /// A copy duplicates the node hierarchy; the implicit copy shared the root between two trees
  Tree(const Tree& other);
  Tree& operator=(const Tree& other);
  Tree(Tree&&) noexcept = default;
  Tree& operator=(Tree&&) noexcept = default;

  bool has_guid() const { return !_guid.empty(); }
  const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
  std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Root node of the tree (nullptr when empty)
  std::shared_ptr<TreeNode> root() const;

  /// Add a node to the tree (parent=nullptr adds as root)
  void add(std::shared_ptr<TreeNode> node, std::shared_ptr<TreeNode> parent = nullptr);

  /// All nodes in the tree (breadth-first from root)
  std::vector<std::shared_ptr<TreeNode>> nodes() const;

  /// Remove a node and return it with its subtree intact
  std::shared_ptr<TreeNode> remove(std::shared_ptr<TreeNode> node);

  /// All nodes without children
  std::vector<std::shared_ptr<TreeNode>> leaves() const;

  /// Traverse from root ("depthfirst"|"breadthfirst", "preorder"|"postorder")
  std::vector<std::shared_ptr<TreeNode>> traverse(const std::string &strategy = "depthfirst", const std::string &order = "preorder") const;

  /// First node with the given name (nullptr if not found)
  std::shared_ptr<TreeNode> get_node_by_name(const std::string &node_name) const;

  /// All nodes with the given name
  std::vector<std::shared_ptr<TreeNode>> get_nodes_by_name(const std::string &node_name) const;

  /// Node with the given guid (nullptr if not found)
  std::shared_ptr<TreeNode> find_node_by_guid(const std::string &node_guid) const;

  /// Reparent a child by guid; false when either node is missing or the child is the root
  bool add_child_by_guid(const std::string &parent_guid, const std::string &child_guid);

  /// Guids of the children of a node by guid (empty if not found)
  std::vector<std::string> get_children_guids(const std::string &node_guid) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Serialization
  // ═══════════════════════════════════════════════════════════════════════════

  nlohmann::ordered_json jsondump() const;
  static Tree jsonload(const nlohmann::json &data);
  std::string file_json_dumps() const;
  static Tree file_json_loads(const std::string& json_string);
  void file_json_dump(const std::string& filename) const;
  static Tree file_json_load(const std::string& filename);
  std::string pb_dumps() const;
  static Tree pb_loads(const std::string& data);
  void pb_dump(const std::string& filename) const;
  static Tree pb_load(const std::string& filename);

  std::string str() const;
};

std::ostream &operator<<(std::ostream &os, const Tree &tree);

} // namespace session_cpp
