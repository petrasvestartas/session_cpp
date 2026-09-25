#pragma once
#include "color.h"
#include <guid.h>
#include <json.h>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace session_cpp {

class Tomb;

// ═══════════════════════════════════════════════════════════════════════════
// TreeNode
// ═══════════════════════════════════════════════════════════════════════════
/// A node of a tree; geometry nodes are named by their object's guid, group nodes by a label.
class TreeNode : public std::enable_shared_from_this<TreeNode> {
    friend class Tree;
    friend class Session;

private:
    mutable std::string _guid; // Lazy guid.
    std::weak_ptr<TreeNode> _parent; // Parent node, empty for the root.
    std::vector<std::shared_ptr<TreeNode>> _children; // Raw child nodes in order, dead ones included.
    bool _dead = false; // Hidden from every public walk.
    std::weak_ptr<Tomb> _tomb; // Weak pin while a record holds it.
    size_t _at = 0; // Raw index in the parent's children.
    bool _queued = false; // Whether Session.sweep holds this parent.
    std::optional<std::pair<size_t, size_t>> _cursor; // (read, write) while a compaction is part way.

public:
    std::string name; // Object guid or group label.
    std::optional<Color> color; // Display colour override.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct a node with a name.
    TreeNode(std::string name = "my_node") : name(name) {}

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    bool has_guid() const {
        return !_guid.empty();
    }

    /// Return the guid, creating it on first access.
    const std::string& guid() const {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return the mutable guid, creating it on first access.
    std::string& guid() {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return whether this node has no parent.
    bool is_root() const;

    /// Return whether this node has no live children.
    bool is_leaf() const;

    /// Return the parent node, or nullptr for the root and for a dead node.
    std::shared_ptr<TreeNode> parent() const;

    /// Return all ancestors from the immediate parent up to the root.
    std::vector<TreeNode*> ancestors() const;

    /// Return all descendants of this node, depth-first.
    std::vector<TreeNode*> descendants() const;

    /// Return the live direct children of this node.
    std::vector<TreeNode*> children() const;

    /// Return whether this node is dead.
    bool is_dead() const;

    /// Return the tomb pinning this node while a record still holds it.
    std::shared_ptr<Tomb> get_tomb() const;

    /// Return whether a compaction of the children is part way.
    bool is_compacting() const;

    /// Return the raw index in the parent's children, dead siblings counted.
    size_t at() const;

    /// Return whether Session.sweep holds this node.
    bool is_queued() const;

    /// Return whether a node is a child, dead or alive.
    bool has_child(const std::shared_ptr<TreeNode>& child) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Mutators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Append a child; a child placed elsewhere moves and leaves the returned dead ghost in its old slot.
    std::shared_ptr<TreeNode> add(std::shared_ptr<TreeNode> child);

    /// Remove a child node and return it, or nullptr when not found; aborts a running compaction.
    std::shared_ptr<TreeNode> remove(std::shared_ptr<TreeNode> child);

    /// Kill or revive this node in O(1); a dead node hides itself and its subtree from every walk.
    void set_dead(bool dead);

    /// Pin this node weakly to a tomb.
    void set_tomb(const std::shared_ptr<Tomb>& tomb);

    /// Mark whether Session.sweep holds this node.
    void set_queued(bool queued);

    /// Exchange the places of two nodes, each into the other's parent and raw slot; their subtrees travel with them.
    static void swap(const std::shared_ptr<TreeNode>& a, const std::shared_ptr<TreeNode>& b);

    /// Purge unpinned dead children for at most work children, resuming where the last call stopped; returns the children examined.
    size_t compact_step(size_t work);

    /// Finish a running compaction, then purge every unpinned dead child.
    void compact();

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Compare by guid.
    bool operator==(const TreeNode& other) const;

    /// Compare by guid.
    bool operator!=(const TreeNode& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Traversal
    // ═══════════════════════════════════════════════════════════════════════════
    /// Traverse from this node ("depthfirst"|"breadthfirst", "preorder"|"postorder").
    std::vector<TreeNode*> traverse(const std::string& strategy = "depthfirst", const std::string& order = "preorder") const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static std::shared_ptr<TreeNode> jsonload(const nlohmann::json& data);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the name and child count.
    std::string str() const;

    /// Return the name, guid and child count.
    std::string repr() const;

private:
    /// Return the raw index of a child, O(1) through its _at, or nullopt when not a child.
    std::optional<size_t> _position(const std::shared_ptr<TreeNode>& child) const;
};

/// Write the node string to a stream.
std::ostream& operator<<(std::ostream& os, const TreeNode& node);

// ═══════════════════════════════════════════════════════════════════════════
// Tree
// ═══════════════════════════════════════════════════════════════════════════
/// A hierarchy of TreeNodes under one root.
class Tree {
private:
    mutable std::string _guid; // Lazy guid.
    std::shared_ptr<TreeNode> _root; // Root node, null when empty.

public:
    std::string name; // Tree name.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty tree with a name.
    Tree(std::string name = "my_tree") : name(name) {}

    /// Copy by duplicating the node hierarchy; the implicit copy shared the root between two trees.
    Tree(const Tree& other);

    /// Copy-assign by duplicating the node hierarchy.
    Tree& operator=(const Tree& other);

    /// Move while preserving the guid.
    Tree(Tree&&) noexcept = default;

    /// Move-assign while preserving the guid.
    Tree& operator=(Tree&&) noexcept = default;

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    bool has_guid() const {
        return !_guid.empty();
    }

    /// Return the guid, creating it on first access.
    const std::string& guid() const {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return the mutable guid, creating it on first access.
    std::string& guid() {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return the root node, or nullptr when empty.
    std::shared_ptr<TreeNode> root() const;

    /// Return all nodes in the tree, breadth-first from the root.
    std::vector<std::shared_ptr<TreeNode>> nodes() const;

    /// Return all nodes without children.
    std::vector<std::shared_ptr<TreeNode>> leaves() const;

    /// Return the first node with the given name, or nullptr when not found.
    std::shared_ptr<TreeNode> get_node_by_name(const std::string& node_name) const;

    /// Return all nodes with the given name.
    std::vector<std::shared_ptr<TreeNode>> get_nodes_by_name(const std::string& node_name) const;

    /// Return the node with the given guid, or nullptr when not found.
    std::shared_ptr<TreeNode> find_node_by_guid(const std::string& node_guid) const;

    /// Return the guids of the children of a node by guid, empty when not found.
    std::vector<std::string> get_children_guids(const std::string& node_guid) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Mutators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Add a node to the tree; a null parent adds it as the root.
    void add(std::shared_ptr<TreeNode> node, std::shared_ptr<TreeNode> parent = nullptr);

    /// Remove a node and return it with its subtree intact.
    std::shared_ptr<TreeNode> remove(std::shared_ptr<TreeNode> node);

    /// Reparent a child by guid; false when either node is missing or the child is the root.
    bool add_child_by_guid(const std::string& parent_guid, const std::string& child_guid);

    // ═══════════════════════════════════════════════════════════════════════════
    // Traversal
    // ═══════════════════════════════════════════════════════════════════════════
    /// Traverse from the root ("depthfirst"|"breadthfirst", "preorder"|"postorder").
    std::vector<std::shared_ptr<TreeNode>> traverse(const std::string& strategy = "depthfirst", const std::string& order = "preorder") const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Tree jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static Tree file_json_loads(const std::string& json_string);

    /// Write to a JSON file.
    void file_json_dump(const std::string& filename) const;

    /// Read from a JSON file.
    static Tree file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static Tree pb_loads(const std::string& data);

    /// Write to a protobuf file.
    void pb_dump(const std::string& filename) const;

    /// Read from a protobuf file.
    static Tree pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the node count and the hierarchy drawn with box-drawing connectors.
    std::string str() const;

    /// Return the tree name and node count.
    std::string repr() const;
};

/// Write the tree string to a stream.
std::ostream& operator<<(std::ostream& os, const Tree& tree);

} // namespace session_cpp
