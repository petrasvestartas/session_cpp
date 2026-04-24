#pragma once
#include "color.h"
#include <algorithm>
#include <fstream>
#include <functional>
#include <guid.h>
#include <iostream>
#include <json.h>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace session_cpp {

// Forward declaration - TreeNode needs to know Tree exists, but not its definition
class Tree;

/**
 * @class TreeNode
 * @brief A node of a tree data structure
 *
 * TreeNodes can represent either:
 * - Geometry nodes: name is set to the geometry's GUID for lookup
 * - Organizational nodes: name is a descriptive string (e.g., "folder", "group")
 */
class TreeNode : public std::enable_shared_from_this<TreeNode> {
  friend class Tree; // Allow Tree to access private members
private:
  mutable std::string _guid;
  std::weak_ptr<TreeNode> _parent;                    ///< Non-owning pointer to parent
  std::vector<std::shared_ptr<TreeNode>> _children;   ///< Owning pointers to children
  std::weak_ptr<Tree> _tree;                          ///< Non-owning pointer to tree

public:
  /// Default / named constructor
  TreeNode(std::string name = "my_node") { this->name = name; }

  /// Lazy GUID accessor (const)
  const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Lazy GUID accessor (mutable)
  std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Node identifier/name. For geometry nodes, this is the geometry's GUID
  std::string name;

  /// Optional display color, used for layer nodes
  std::optional<Color> color;

  /// Simple string form (like Python __str__)
  std::string str() const;

  /// Equality (compares guid)
  bool operator==(const TreeNode &other) const;

  /// Inequality (compares guid)
  bool operator!=(const TreeNode &other) const;

  ///////////////////////////////////////////////////////////////////////////////////////////
  // JSON
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Serialize to ordered JSON object
  nlohmann::ordered_json jsondump() const;

  /// Deserialize from JSON object
  static std::shared_ptr<TreeNode> jsonload(const nlohmann::json &data);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Details
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// True if this node has no parent
  bool is_root() const;

  /// True if this node has no children
  bool is_leaf() const;

  /// Get the owning tree (nullptr if not attached to a tree)
  Tree *tree() const;

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
  std::vector<TreeNode *> traverse(const std::string &strategy = "depthfirst",
                                   const std::string &order = "preorder") const;
};

/// Stream output operator for tree node
std::ostream &operator<<(std::ostream &os, const TreeNode &node);

/**
 * @class Tree
 * @brief A hierarchical data structure with parent-child relationships.
 */
class Tree {
  friend class TreeNode;

public:
  /// Lazy GUID accessor (const)
  const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Lazy GUID accessor (mutable)
  std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Tree identifier/name
  std::string name = "my_tree";

private:
  mutable std::string _guid;
  std::shared_ptr<TreeNode> _root; ///< Root node of the tree

public:
  /// Default / named constructor
  Tree(std::string name = "my_tree") { this->name = name; }

  /// Simple string form (like Python __str__)
  std::string str() const;

  ///////////////////////////////////////////////////////////////////////////////////////////
  // JSON
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Serialize to ordered JSON object
  nlohmann::ordered_json jsondump() const;

  /// Deserialize from JSON object
  static Tree jsonload(const nlohmann::json &data);

  /// Convert to JSON string
  std::string file_json_dumps() const;

  /// Load from JSON string
  static Tree file_json_loads(const std::string& json_string);

  /// Write JSON to file
  void file_json_dump(const std::string& filename) const;

  /// Read JSON from file
  static Tree file_json_load(const std::string& filename);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Protobuf
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Convert to protobuf binary string
  std::string pb_dumps() const;

  /// Load from protobuf binary string
  static Tree pb_loads(const std::string& data);

  /// Write protobuf to file
  void pb_dump(const std::string& filename) const;

  /// Read protobuf from file
  static Tree pb_load(const std::string& filename);

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Details
  ///////////////////////////////////////////////////////////////////////////////////////////

  /// Get the root node of the tree
  std::shared_ptr<TreeNode> root() const;

  /// Add a node to the tree (parent=nullptr adds as root)
  void add(std::shared_ptr<TreeNode> node,
           std::shared_ptr<TreeNode> parent = nullptr);

  /// All nodes in the tree (breadth-first from root)
  std::vector<std::shared_ptr<TreeNode>> nodes() const;

  /// Remove a node from the tree
  std::shared_ptr<TreeNode> remove(std::shared_ptr<TreeNode> node);

  /// All leaf nodes (nodes without children)
  std::vector<std::shared_ptr<TreeNode>> leaves() const;

  /// Traverse from root ("depthfirst"|"breadthfirst", "preorder"|"postorder")
  std::vector<std::shared_ptr<TreeNode>>
  traverse(const std::string &strategy = "depthfirst",
           const std::string &order = "preorder") const;

  /// First node with the given name (nullptr if not found)
  std::shared_ptr<TreeNode> get_node_by_name(const std::string &node_name) const;

  /// All nodes with the given name
  std::vector<std::shared_ptr<TreeNode>>
  get_nodes_by_name(const std::string &node_name) const;

  /// Find a node by its GUID (nullptr if not found)
  std::shared_ptr<TreeNode> find_node_by_guid(const std::string &node_guid) const;

  /// Reparent a child by GUID; returns true if both nodes were found
  bool add_child_by_guid(const std::string &parent_guid,
                         const std::string &child_guid);

  /// GUIDs of children of a node by GUID (empty if not found or no children)
  std::vector<std::string> get_children_guids(const std::string &node_guid) const;

  /// Print the hierarchy to stdout
  void print_hierarchy() const;
};

/// Stream output operator for tree
std::ostream &operator<<(std::ostream &os, const Tree &tree);

} // namespace session_cpp
