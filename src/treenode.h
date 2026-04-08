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

} // namespace session_cpp
