#pragma once
#include "color.h"
#include "graph.h"
#include "interaction.h"
#include "objects.h"
#include "tree.h"
#include "xform.h"
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace session_cpp {

class Session;

inline constexpr int CAPACITY = 64; // Committed transactions kept; past it the oldest is dropped.
inline constexpr size_t BUDGET = 256u << 20; // Bytes the stacks may pin; past it the oldest is dropped.
inline constexpr size_t RECORD = 256; // Bytes one record costs on top of what it pins.

/// A deep copy that keeps the guid, which `duplicate()` and most copy constructors would mint anew.
Item clone(const Item& obj);

/// A deep copy of geometry that keeps its guid and type, element feature guids included.
Geometry clone(const Geometry& obj);

/// A copy of features that keeps each guid, which the ElementFeature copy would mint anew.
std::vector<ElementFeature> clone(const std::vector<ElementFeature>& features);

/// An estimate of the bytes an item pins while a record holds it, O(1) from its container lengths.
size_t weight(const Item& item);

// ═══════════════════════════════════════════════════════════════════════════
// Records
// ═══════════════════════════════════════════════════════════════════════════
/// One dead or revivable entity: where it lives and what it parked while dead; slots and nodes pin it weakly, records strongly.
class Tomb {
public:
    std::string collection;          // The Objects list of its slot, "" for a node-only tomb.
    bool definition = false;         // Whether the slot is in Session::definitions.
    size_t slot = 0;                 // Its raw slot, moved by compaction.
    std::shared_ptr<TreeNode> node;  // Its tree node, nullptr for a slot-only tomb.
    std::optional<Vertex> vertex;    // Its graph vertex while dead.
    std::vector<Edge> edges;         // Its incident edges while dead.
    std::optional<Xform> xform;      // Its local transform while dead.
    std::map<std::string, std::vector<std::shared_ptr<Interaction>>> interactions; // Its edges' interactions while dead, by edge guid.

    /// Construct a tomb with nothing parked; always held through std::make_shared.
    Tomb(const std::string& collection, bool definition, size_t slot, std::shared_ptr<TreeNode> node);
};

/// An object added or removed: the tomb that flips it and where its node sits.
class Tombstone {
public:
    std::string kind = "";                  // "add" or "remove".
    std::string guid;                       // The object's guid.
    std::string collection;                 // The Objects list it lives in, or "definitions".
    std::optional<std::string> parent_guid; // Name of its tree parent, nullopt when it has no node.
    int index;                              // Its raw index among the parent's children at record time, a hint.
    std::shared_ptr<TreeNode> node;         // Its tree node, for adds too; nullptr when it has none.
    std::shared_ptr<Tomb> tomb;             // The tomb undo and redo flip.

    /// Construct from every field of the record.
    Tombstone(
        const std::string& guid,
        const std::string& collection,
        const std::optional<std::string>& parent_guid,
        int index,
        std::shared_ptr<TreeNode> node,
        std::shared_ptr<Tomb> tomb
    );

    /// Return a string representation of the record.
    std::string str() const;

    /// Return a string representation of the record for debugging.
    std::string repr() const;
};

/// An object entered the session; undo kills its tomb, redo revives it.
class AddOp : public Tombstone {
public:
    /// Construct an add record from every field of the record.
    AddOp(
        const std::string& guid,
        const std::string& collection,
        const std::optional<std::string>& parent_guid,
        int index,
        std::shared_ptr<TreeNode> node,
        std::shared_ptr<Tomb> tomb
    );
};

/// An object left the session; undo revives its tomb, redo kills it.
class RemoveOp : public Tombstone {
public:
    /// Construct a remove record from every field of the record.
    RemoveOp(
        const std::string& guid,
        const std::string& collection,
        const std::optional<std::string>& parent_guid,
        int index,
        std::shared_ptr<TreeNode> node,
        std::shared_ptr<Tomb> tomb
    );
};

/// The entry a replace was taken on: an object by its tree node at record time (nullptr outside the tree) or a definition by its slot.
class Entry {
public:
    bool definition;                // Whether the entry is a definition.
    std::shared_ptr<TreeNode> node; // An object's tree node at record time; nullptr outside the tree or for a definition.
    size_t slot;                    // A definition's slot; 0 for an object.

    /// Construct an object entry from its node or a definition entry from its slot.
    Entry(bool definition, std::shared_ptr<TreeNode> node, size_t slot);
};

/// The object or definition under `guid` was swapped: the stored pointers before and after, never copies.
class ReplaceOp {
public:
    std::string kind = "replace"; // Always "replace".
    std::string guid;             // The entry's guid.
    Item before;                  // The entry before the swap.
    Item after;                   // The entry after the swap.
    Entry entry;                  // The entry the swap was taken on.

    /// Construct from the guid, the before and after items and the entry.
    ReplaceOp(const std::string& guid, const Item& before, const Item& after, Entry entry);

    /// Return a string representation of the record.
    std::string str() const;

    /// Return a string representation of the record for debugging.
    std::string repr() const;
};

/// The local transform under `guid` changed; nullopt on either side means "none set".
class XformOp {
public:
    std::string kind = "xform";     // Always "xform".
    std::string guid;               // The object's guid.
    std::optional<Xform> before;    // Transform before the change.
    std::optional<Xform> after;     // Transform after the change.
    std::shared_ptr<TreeNode> node; // The entry's tree node at record time; nullptr for a group or an object outside the tree.

    /// Construct from the guid, the before and after transforms and the entry's node.
    XformOp(
        const std::string& guid,
        const std::optional<Xform>& before,
        const std::optional<Xform>& after,
        std::shared_ptr<TreeNode> node
    );

    /// Return a string representation of the record.
    std::string str() const;

    /// Return a string representation of the record for debugging.
    std::string repr() const;
};

/// A tree node added, removed, moved, renamed or recoloured: its state before and after.
class TreeOp {
public:
    std::string kind = "tree";         // Always "tree".
    std::string guid;                  // The node name at record time.
    std::shared_ptr<TreeNode> node;    // The node itself.
    std::shared_ptr<Tomb> tomb;        // Node-only; pins the ghost of a move, else the node.
    std::shared_ptr<TreeNode> ghost;   // The dead ghost a move left in the old slot, nullptr otherwise.
    std::string name_before;           // Name before.
    std::string name_after;            // Name after.
    std::optional<Color> color_before; // Colour before.
    std::optional<Color> color_after;  // Colour after.
    bool dead_before;                  // Whether it was dead or absent before.
    bool dead_after;                   // Whether it is dead after.

    /// Construct from every field of the record.
    TreeOp(
        const std::string& guid,
        std::shared_ptr<TreeNode> node,
        std::shared_ptr<Tomb> tomb,
        std::shared_ptr<TreeNode> ghost,
        const std::string& name_before,
        const std::string& name_after,
        std::optional<Color> color_before,
        std::optional<Color> color_after,
        bool dead_before,
        bool dead_after
    );

    /// Return a string representation of the record.
    std::string str() const;

    /// Return a string representation of the record for debugging.
    std::string repr() const;
};

/// Any one recorded op.
using Op = std::variant<AddOp, RemoveOp, ReplaceOp, XformOp, TreeOp>;

/// One undoable step: a label, the ops it made in the order they happened, and the bytes they pin.
class Transaction {
public:
    std::string label;   // What the step did.
    std::vector<Op> ops; // Ops in the order they happened.
    size_t bytes = 0;    // Bytes its records pin.

    /// Construct an empty transaction with a label.
    Transaction(std::string label = "my_transaction");

    /// Return a string representation of the transaction.
    std::string str() const;

    /// Return a string representation of the transaction for debugging.
    std::string repr() const;
};

// ═══════════════════════════════════════════════════════════════════════════
// History
// ═══════════════════════════════════════════════════════════════════════════
/// CAD-style undo/redo over a Session, in memory only: records flip tombs in place, every save purges them.
class History {
public:
    std::vector<Transaction> undo_stack; // Committed transactions, oldest first; capped at CAPACITY and budget.
    std::vector<Transaction> redo_stack; // Undone transactions, cleared the moment a new transaction commits.
    std::optional<Transaction> current;  // The open transaction, nullopt between commit and the next begin.
    size_t bytes = 0;                    // Bytes pinned by both stacks and the open transaction.
    size_t budget = BUDGET;              // Bytes the stacks may pin before the oldest is dropped.
    size_t dropped = 0;                  // Ops dropped since the last purge cycle began, unrecorded kills included.

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether a committed transaction can be undone.
    bool can_undo() const;

    /// Return whether an undone transaction can be redone.
    bool can_redo() const;

    /// Return the number of committed transactions.
    int depth() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Transactions
    // ═══════════════════════════════════════════════════════════════════════════
    /// Open a transaction; an already open one is committed first so no op is lost.
    void begin(const std::string& label);

    /// Close the open transaction. An empty one is dropped; a real one clears redo and trims the oldest past the caps.
    void commit();

    /// Append an op pinning `bytes` to the open transaction; a no-op when none is open.
    void record(Op op, size_t bytes);

    /// Revert the open transaction's ops in reverse and drop it, leaving both stacks as they are; false when none is open.
    bool abort(Session& session);

    /// Revert the newest transaction, ops in reverse order, and park it for redo.
    bool undo(Session& session);

    /// Re-apply the newest undone transaction, ops in their original order.
    bool redo(Session& session);

    /// Drop every transaction, open or committed; what they pinned is purgeable now.
    void clear();

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return a string representation of the history.
    std::string str() const;

    /// Return a string representation of the history for debugging.
    std::string repr() const;

private:
    /// Bytes pinned by both stacks.
    size_t _pinned() const;

    /// Undo one op against the session.
    void _revert(const Op& op, Session& session);

    /// Redo one op against the session.
    void _apply(const Op& op, Session& session);
};

} // namespace session_cpp
